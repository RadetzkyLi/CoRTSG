"""
Camera
"""
import os
import numpy as np
import weakref
import time


from coriskyscene.data_collection.sensors.base_sensor import BaseSensor
from coriskyscene.data_collection.sensors.sensor_util import \
    spawn_point_estimation_cav, spawn_point_estimation_rsu, get_camera_intrinsic

class RgbCamera(BaseSensor):
    def __init__(self, agent_id, parent_content, world, config, global_position):
        """
        Parameters
        ----------
        agent_id: int
            Parent actor's id. 
        
        parent_content: dict|None
            If not None, will be {"actor": CARLA.Actor, "map_name": str}. The actor is the 
            parent to which the lidar will attach.
        
        world:
            CARLA world
        
        config: dict
            Lidar revelant config
        
        global_position: carla.Transform|None
            Spawn position of lidar
        
        """
        assert parent_content or global_position, "`parent_content` and `global_position` can't "\
            "be None at the same time!"
        
        parent, map_name = parent_content['actor'], parent_content['map_name']
        super().__init__(agent_id, parent, world, config, global_position)
        self.agent_id = agent_id
        relative_position = config['relative_pose']
        self.name = "camera" + relative_position.capitalize()

        if parent is not None:
            world = parent.get_world()

        camera_bp = self.get_blueprint_from_config(world.get_blueprint_library(), config)
        if global_position:
            self.sensor = world.spawn_actor(camera_bp, global_position)
        else:
            spawn_point = self.spawn_point_estimation(parent, map_name, relative_position)
            self.sensor = world.spawn_actor(camera_bp, spawn_point, attach_to=parent)

        # camera data
        self.image = None
        self.timestamp = None
        self.frame = None
        self.transform = None

        weak_self = weakref.ref(self)
        self.sensor.listen(lambda event: RgbCamera._on_camera_event(weak_self, event))


    @staticmethod
    def get_blueprint_from_config(bp_lib, config={}):
        default_config = {
            "image_size_x": 800,
            "image_size_y": 600,
            "fov": 110
        }
        default_config.update(config)
        blueprint = bp_lib.find("sensor.camera.rgb")
        for k,v in default_config.items():
            if blueprint.has_attribute(k):
                blueprint.set_attribute(k, str(v))
        return blueprint
    
    @staticmethod
    def spawn_point_estimation(actor, map_name, relative_position):
        """
        Parameters
        ----------
        actor: carla.Actor
            Sensor's parent

        map_name: str
            Name of the used CARLA map.

        relative_position: str
            Position of the sensor relative to its parent

        global_position: carla.Transform
            Sensor's global position
        """
        if actor.type_id.startswith("vehicle"):
            spawn_point = spawn_point_estimation_cav(actor, "camera", relative_position)
        elif actor.type_id.startswith("traffic.traffic_light"):
            spawn_point = spawn_point_estimation_rsu(actor, "camera", map_name, relative_position)
        else:
            raise ValueError("Actor of type {0} is not supported to be sensor's parent!".format(actor.type_id))
            
        return spawn_point
    
    def get_intrinsic(self):
        return get_camera_intrinsic(self.sensor)
    
    
    def data_dump(self, output_folder, cur_timestamp):
        while self.image is None:
            time.sleep(0.1)
            continue
        # save_path = os.path.join(output_folder, "{0:06}_{1}.jpg".format(cur_timestamp, self.name))
        save_path = os.path.join(output_folder, "{0:06}_{1}.png".format(cur_timestamp, self.name))
        self.image.save_to_disk(save_path)
        
    @staticmethod
    def _on_camera_event(weak_self, event):
        self = weak_self()
        if not self:
            return
        self.image = event

        self.frame = event.frame
        self.timestamp = event.timestamp
        self.transform = event.transform
