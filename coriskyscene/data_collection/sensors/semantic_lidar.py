"""
This is mainly used to filter out objects that is not in the sight
of LiDAR.
"""
import os
import sys
import weakref
import carla
import time
import numpy as np

from coriskyscene.data_collection.sensors.base_sensor import BaseSensor
from coriskyscene.data_collection.sensors.sensor_util import \
    spawn_point_estimation_cav, spawn_point_estimation_rsu
from coriskyscene.common_utils.yaml_util import save_yaml

class SemanticLidar(BaseSensor):
    def __init__(self, agent_id, parent_content, world, config, global_position):
        assert parent_content or global_position, "`parent_content` and `global_position` can't "\
            "be None at the same time!"
        parent, map_name = parent_content['actor'], parent_content['map_name']
        super().__init__(agent_id, parent, world, config, global_position)
        
        if parent is not None:
            world = parent.get_world()

        self.agent_id = agent_id
        self.name = 'semantic_lidar'
        blueprint = self.get_blueprint_from_config(world.get_blueprint_library(), config)
       
        if global_position:
            spawn_point = global_position
            self.sensor = world.spawn_actor(blueprint, spawn_point)
        else:
            spawn_point = self.spawn_point_estimation(parent, map_name)
            self.sensor = world.spawn_actor(
                blueprint, spawn_point, attach_to=parent)
        
        # lidar data
        self.points = None
        self.obj_idx = None
        self.obj_tag = None
        self.obj_nop = None  # number of points

        self.timestamp = None
        self.frame = 0
        self.transform = None

        weak_self = weakref.ref(self)
        self.sensor.listen(
            lambda event: SemanticLidar._on_data_event(
                weak_self, event))
        
        # waiting rendering
        self.waiting_interval = 1.0   # seconds
        self.max_waiting_time = 30.0  # seconds
        
    @staticmethod
    def get_blueprint_from_config(bp_lib, config={}):
        default_config = {
            "upper_fov": 5,
            "lower_fov": -25,
            "channels": 64,
            "range": 120,
            "points_per_second": 1300000,
            "rotation_frequency": 20,
            "relative_pose": "top"
        }
        default_config.update(config)

        blueprint = bp_lib.find('sensor.lidar.ray_cast_semantic')
        # set attribute based on the configuration
        for k,v in default_config.items():
            if blueprint.has_attribute(k):
                blueprint.set_attribute(k, str(v))
        return blueprint
    
    @staticmethod
    def spawn_point_estimation(actor, map_name):
        if actor.type_id.startswith("vehicle"):
            spawn_point = spawn_point_estimation_cav(actor, "lidar")
        elif actor.type_id.startswith("traffic.traffic_light"):
            spawn_point = spawn_point_estimation_rsu(actor, "lidar", map_name)
        else:
            raise ValueError("Actor of type {0} is not supported to be sensor's parent!".format(actor.type_id))
        return spawn_point

    @staticmethod
    def _on_data_event(weak_self, event):
        """Semantic Lidar  method"""
        self = weak_self()
        if not self:
            return

        # shape:(n, 6)
        data = np.frombuffer(event.raw_data, dtype=np.dtype([
            ('x', np.float32), ('y', np.float32), ('z', np.float32),
            ('CosAngle', np.float32), ('ObjIdx', np.uint32),
            ('ObjTag', np.uint32)]))

        # (x, y, z)
        self.points = np.array([data['x'], data['y'], data['z']]).T
        self.obj_tag = np.array(data['ObjTag'])
        self.obj_idx = np.array(data['ObjIdx'])

        # label 10 is the vehicle, label 4 is the pedestrian
        tags = {
            4: "pedestrian",
            10: "vehicle"
        }
        object_nop = []
        for tag,cls in tags.items():
            object_idx = self.obj_idx[self.obj_tag == tag]
            # each individual instance id
            object_unique_id = list(np.unique(object_idx))
            for obj_id in object_unique_id:
                object_nop.append({
                    "object_id": int(obj_id),
                    "visible_points": int(np.sum(object_idx == obj_id)),
                    "category": cls})
        self.obj_nop = object_nop

        self.data = data
        self.frame = event.frame
        self.timestamp = event.timestamp
        self.transform = event.transform

    def tick(self):
        total_waiting_time = 0.0
        while self.obj_idx is None or self.obj_tag is None or \
                self.obj_idx.shape[0] != self.obj_tag.shape[0]:
            
            time.sleep(self.waiting_interval)
            total_waiting_time += self.waiting_interval
            if total_waiting_time > self.max_waiting_time:
                raise ValueError("Waiting data timeout! Max waiting time: {0}s".format(self.max_waiting_time))
            continue

        return self.obj_nop
    
    def data_dump(self, output_folder, cur_timestamp):
        # record objects in the view
        save_path = os.path.join(output_folder, "{0:06}_view.yaml".format(cur_timestamp))
        save_yaml({"visible_objects": self.obj_nop}, save_path)
