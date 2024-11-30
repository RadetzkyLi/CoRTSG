"""
This is a virtual sensor which records global transform and speed of 
all traffic participants (vehicles, walkers)
"""
import os

from coriskyscene.data_collection.sensors.base_sensor import BaseSensor
from coriskyscene.common_utils.yaml_util import save_yaml

class TrafficState(BaseSensor):
    def __init__(self, agent_id, vehicle, world, config, global_position):
        super().__init__(agent_id, vehicle, world, config, global_position)

        self.world = world


    def data_dump(self, output_folder, cur_timestamp):
        """
        Retrive all info and save them
        """
        # gather info
        actor_info_dict = {}
        for actor in self.world.get_actors().filter('[vehicle,walker].*'):
            pass
        save_path = os.path.join(output_folder, "{0:06}_view.yaml".format(cur_timestamp))
        save_yaml(self.obj_nop, save_path)
