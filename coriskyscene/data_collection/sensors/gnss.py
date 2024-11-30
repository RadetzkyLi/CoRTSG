"""
GNSS
"""
import weakref

from coriskyscene.data_collection.sensors.base_sensor import BaseSensor
from coriskyscene.data_collection.sensors.sensor_util import \
    spawn_point_estimation_cav, spawn_point_estimation_rsu


class GNSS(BaseSensor):
    def __init__(self, agent_id, parent_content, world, config, global_position):
        parent, map_name = parent_content['actor'], parent_content['map_name']
        assert parent, "GNSS must attach to a actor!"
        
        super().__init__(agent_id, parent, world, config, None)
        self.agent_id = agent_id
        self.name = "gnss"

        world = parent.get_world()

        gnss_bp = self.get_blueprint_from_config(world.get_blueprint_library(), config)
        spawn_point = self.spawn_point_estimation(parent, map_name)
        self.sensor = world.spawn_actor(gnss_bp, spawn_point, attach_to=parent)

        # gnss data
        self.transform = None # transform of lidar
        self.gnss_position = None
        self.timestamp = None
        self.frame = None

        weak_self = weakref.ref(self)
        self.sensor.listen(lambda event: GNSS._on_gnss_event(weak_self, event))

    @staticmethod
    def get_blueprint_from_config(bp_lib, config={}):
        gnss_bp = bp_lib.find('sensor.other.gnss')
        default_config = {
            "noise_lon_stddev": 0.0
        }
        default_config.update(config)
        for k,v in default_config.items():
            if gnss_bp.has_attribute(k):
                gnss_bp.set_attribute(k, str(v))
        
        return gnss_bp

    @staticmethod
    def spawn_point_estimation(actor, map_name):
        if actor.type_id.startswith("vehicle"):
            spawn_point = spawn_point_estimation_cav(actor, "gnss")
        elif actor.type_id.startswith("traffic.traffic_light"):
            spawn_point = spawn_point_estimation_rsu(actor, "gnss", map_name)
        else:
            raise ValueError("Actor of type {0} is not supported to be sensor's parent!".format(actor.type_id))
        return spawn_point
    
    @staticmethod
    def _on_gnss_event(weak_self, event):
        self = weak_self()
        if not self:
            return
        
        self.frame = event.frame
        self.timestamp = event.timestamp
        self.gnss_position = {
            "lon": event.longitude,
            "lat": event.latitude,
            "alt": event.altitude
        }
        self.transform = event.transform