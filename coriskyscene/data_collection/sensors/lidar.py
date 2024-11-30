"""
LiDAR sensor
"""
import os
import weakref

import numpy as np
import open3d as o3d

from coriskyscene.data_collection.sensors.base_sensor import BaseSensor
from coriskyscene.data_collection.sensors.sensor_util import \
    spawn_point_estimation_cav, spawn_point_estimation_rsu

class Lidar(BaseSensor):
    def __init__(self, agent_id, parent_content, world, config, global_position):
        """
        Parameters
        ----------
        agent_id: int
            Parent actor's id. 
        parent_content: dict|None
            If not None, will be {"actor": CARLA.Actor, "map_name": str}. The actor 
            is the parent to which the lidar will attach.
        world:
            CARLA world
        config: dict
            Lidar config
        global_position: carla.Transform
            Spawn position of lidar
        
        """
        assert parent_content or global_position, "`parent_content` and `global_position`"\
            " can't be None at the same time!"
        parent, map_name = parent_content['actor'], parent_content['map_name']
        super().__init__(agent_id, parent, world, config, global_position)
        
        if parent is not None:
            world = parent.get_world()

        self.agent_id = agent_id
        self.name = 'lidar'

        blueprint = self.get_blueprint_from_config(world.get_blueprint_library(), config)
        if global_position:
            spawn_point = global_position
        else:
            spawn_point = self.spawn_point_estimation(parent, map_name)
        
        if parent is not None:
            self.sensor = world.spawn_actor(
                blueprint, spawn_point, attach_to=parent)
        else:
            self.sensor = world.spawn_actor(blueprint, spawn_point)


        # lidar data
        self.points = None
        self.intensity_values = None
        self.timestamp = None
        self.frame = 0
        self.transform = None

        weak_self = weakref.ref(self)
        self.sensor.listen(
            lambda event: Lidar._on_data_event(
                weak_self, event))

    @staticmethod
    def get_blueprint_from_config(bp_lib, config={}):
        """if config is empty dict, default setting will be used"""
        default_config = {
            "channels": 64,
            "dropoff_general_rate": 0.1,
            "dropoff_intensity_limit": 0.7,
            "dropoff_zero_intensity": 0.15,
            "lower_fov": -25,
            "upper_fov": 2,
            "horizontal_fov": 360.0,
            "noise_stddev": 0.02,
            "points_per_second": 1300000,
            "range": 120,
            "rotation_frequency": 20
        }
        default_config.update(config)
        blueprint = bp_lib.find('sensor.lidar.ray_cast')
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
        """Lidar  method"""
        self = weak_self()
        if not self:
            return

        # shape:(n, 4)
        data = np.frombuffer(event.raw_data, dtype=np.dtype([
            ('x', np.float32), ('y', np.float32), ('z', np.float32),
            ('intensity', np.float32)]))

        # (x, y, z, intensity)
        self.points = np.array([data['x'], data['y'], data['z']]).T
        self.intensity_values = np.array(data['intensity'])
        
        self.data = data
        self.frame = event.frame
        self.timestamp = event.timestamp
        self.transform = event.transform

    @staticmethod
    def save_point_cloud(save_path, points, intensity_values=None):
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points)
        
        if intensity_values is not None:
            colors = np.zeros((len(points), 3)) # Set default color to black
            colors[:, 0] = intensity_values  # Set color intensity based on intensity values
            pcd.colors = o3d.utility.Vector3dVector(colors)  

        # save
        o3d.io.write_point_cloud(save_path, pcd)


    def data_dump(self, output_folder, cur_timestamp):
        # save point cloud in .pcd format
        save_path = os.path.join(output_folder, "{0:06}.pcd".format(cur_timestamp))
        # pcd = o3d.geometry.PointCloud()
        # pcd.points = o3d.utility.Vector3dVector(self.points)
        # o3d.io.write_point_cloud(save_path, pcd)
        self.save_point_cloud(save_path, self.points, self.intensity_values)
    