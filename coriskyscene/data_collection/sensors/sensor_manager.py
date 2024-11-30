# -*- coding: utf-8 -*-
# Author: Runsheng Xu <rxx3386@ucla.edu>
# Modified by: Rongsong Li <rongsong.li@qq.com>
# License: TDG-Attribution-NonCommercial-NoDistrib
# Desc: Sensor Manager for each cav

import os
import sys
import importlib
from collections import OrderedDict

from coriskyscene.data_collection.sensors.generate_annos import generate_anno_for_agent
from coriskyscene.common_utils.yaml_util import save_yaml
from coriskyscene.data_collection.util.id_helper import IdHelper

class SensorManager:
    """
    The manager controls all sensor data streaming and dumping for each cav.
    
    Parameters
    ----------
    agent_id : str
        The cav's original id. It can be named by yourself.

    vehicle_content : dict
        The content of the cav. E.g.: 
        {
            "cur_pose": carla.Transform,
            "model": 'vehicle.lincoln.mkz_2017',  # str
            "color": '0, 0, 255',  # str
            "actor": carla.Actor,
            "actor_id": carla.Actor.id,
            "cur_count": cur_timestamp 
        }

    world : carla.World
        Carla simulation server object.

    config_yaml : dict
        Configurations for the sensor manager.

    output_root : str
        Output directory for data dumping.

    Attributes
    ----------
    sensor_list : list
        A list of sensors to dump/visualize data.
    """
    def __init__(self, 
                 agent_id,
                 agent_content,
                 world, 
                 config_yaml, 
                 output_root):

        self.agent_id = agent_id
        self.output_root = output_root
        self.parent = agent_content['actor']
        self.map_name = agent_content['map_name']
        self.world = world
        self.sensor_list = []
        # this is used to gather the meta information return from sensors
        self.sensor_meta = OrderedDict()

        for sensor_content in config_yaml['sensor_list']:
            sensor = None
            sensor_name = sensor_content['name']

            # find the corresponding path
            sensor_filename = "coriskyscene.data_collection.sensors." + sensor_name
            sensor_lib = importlib.import_module(sensor_filename)
            target_sensor_name = sensor_name.replace('_', '')

            # the sensor corresponding class has to have the same
            # name pattern as the file
            for name, cls in sensor_lib.__dict__.items():
                if name.lower() == target_sensor_name.lower():
                    sensor = cls

            assert sensor is not None, 'The sensor class name has to be the' \
                                       'same as the file name. e.g. ' \
                                       'bev_semantic_camera -> ' \
                                       'BevSemanticCamera'
            # todo: rsu is not considered yet
            sensor_instance = sensor(self.agent_id,
                                     agent_content,
                                     self.world,
                                     sensor_content['args'],
                                     None)
            self.sensor_list.append(sensor_instance)

    def reset_output_root_to(self, new_root:str):
        self.output_root = new_root

    def run_step(self, cur_timestamp, agent_id_list, actor_speed):
        # step1: save raw data
        for sensor_instance in self.sensor_list:
            sensor_name = sensor_instance.name
            sensor_instance.visualize_data()

            meta_info = sensor_instance.tick()
            self.sensor_meta.update({sensor_name: meta_info})

            # for data dumping
            output_folder = os.path.join(self.output_root,
                                         self.agent_id)
            if not os.path.exists(output_folder):
                os.makedirs(output_folder)
            sensor_instance.data_dump(output_folder,
                                      cur_timestamp)
            
        # step2: save annotation info (including connection info)
        actor_id = IdHelper.agent_id_to_actor_id(self.agent_id)
        actor_id_list = [IdHelper.agent_id_to_actor_id(el) for el in agent_id_list]
        actor_list = [self.world.get_actor(el) for el in actor_id_list if el is not None]
        if actor_id:
            anno_dict = generate_anno_for_agent(self.world, actor_id, self, actor_list, actor_speed)
            save_path = os.path.join(output_folder, "{0:06}.yaml".format(cur_timestamp))
            save_yaml(anno_dict, save_path)

    def destroy(self):
        for sensor_instance in self.sensor_list:
            sensor_instance.destroy()

