#!/usr/bin/env python
# -*- coding: utf-8 -*-
'''
@File    :   carla_simulation.py
@Date    :   2024-01-10
@Author  :   Rongsong Li <rongsong.li@qq.com>
@Version :   1.0
@Desc    :   Render traffic scene using simulator CARLA
'''

import os
import time
import psutil
import shutil
import logging
import carla

from coriskyscene.data_collection.util.id_helper import IdHelper
from coriskyscene.data_collection.sensors.sensor_manager import SensorManager
from coriskyscene.data_collection.sensors.generate_annos import generate_anno_for_map
from coriskyscene.common_utils.yaml_util import load_yaml, save_yaml, append_to_yaml
from coriskyscene.common_utils.misc import bcolors


INVALID_ACTOR_ID = -1

class CarlaSimulation(object):
    """
    CarlaSimulation is responsible for the management of the carla simulation.
    """
    def __init__(self, cfg_file, host, port, step_length, map_name:str=None) -> None:
        self.cfg_file = cfg_file
        self.carla_config = load_yaml(cfg_file)
        self.carla_config['world']['client_host'] = host
        self.carla_config['world']['port'] = port
        self.carla_config['world']['fixed_delta_seconds'] = step_length
        
        # load world
        if map_name:
            self.carla_config['world']['map_name'] = map_name
        
        # sensors
        # structure: {"agent_id": {"sensor_manager": xxx, }}
        self.sensor_dict = dict()  

        # speed, format: {actor_id: [lon_v, lat_v]}
        self.actor_speed_dict = {}

    def init_carla(self):
        """Initialize """
        carla_ready = False
        while not carla_ready:
            carla_ready = self.is_carla_server_running()
            time.sleep(5)

        self.client = carla.Client(self.carla_config['world']['client_host'], 
                                   self.carla_config['world']['port'])
        self.client.set_timeout(20.0)

        map_name = self.carla_config['world']['map_name']

        try:
            world = self.client.load_world(map_name)
        except RuntimeError as err:
            logging.error(err)
            print(f"{bcolors.FAIL} %s is not found in your CARLA repo!{bcolors.ENDC}"%map_name)
            return False

        self.current_map = world.get_map()

        self.current_frame = 0
        
        # Configuring carla simulation in sync mode
        self.original_settings = world.get_settings()
        new_settings = world.get_settings()
        new_settings.synchronous_mode = True
        new_settings.fixed_delta_seconds = self.carla_config['world']['fixed_delta_seconds']
        world.apply_settings(new_settings)
        self.world = world

        self.has_spawned_rsu = False
        # get all traffic light
        self.used_tls = {}  # {landmark_id: traffic_light_id}
        self._tls = {}      # {landmark_id: traffic_light_actor}
        for landmark in self.current_map.get_all_landmarks_of_type('1000001'):
            if landmark.id != '':
                traffic_light = self.world.get_traffic_light(landmark)
                if traffic_light is None:
                    logging.warning('Landmark %s is not linked to any traffic light', landmark.id)
                    continue
                self._tls[landmark.id] = traffic_light
                traffic_light.set_state(carla.TrafficLightState.Green)

        # For layered map, clear parked vehicles
        if map_name.endswith("_Opt"):
            self.world.unload_map_layer(carla.MapLayer.ParkedVehicles)

        return True
    
    def relaunch_carla_server(self, gpu_id:int):
        """After running for a long time, CARLA may encounter some problem and
        becomes slow. It's necessary to restart it.
        
        """
        # steps: close --> kill --> launch
        self.close()

        os.system("pkill -9 CarlaUE4")
        time.sleep(5)

        commands = "${CARLA_HOME}/CarlaUE4.sh -carla-server -RenderOffScreen -graphicsadapter={gpu_id} > ${CARLA_HOME}/carla.log 2>&1 &"
        commands = "export CUDA_VISIBLE_DEVICES={gpu_id}; export SDL_HINT_CUDA_DEVICE={gpu_id}; " + commands  

        os.system(commands)
        time.sleep(20)

    def kill_carla_server(self):
        os.system("pkill -9 CarlaUE4")
        time.sleep(5)

    @staticmethod
    def is_carla_server_running():
        for process in psutil.process_iter(attrs=['pid', 'name']):
            if process.info['name'] == "CarlaUE4-Linux-Shipping":
                return True
        return False
        
    def get_current_overview(self):
        filter_dict = {
            "vehicle": "vehicle.*",
            "walker": "walker.*",
            "camera": "sensor.camera.*",
            "lidar": "sensor.lidar.*"
        }
        info_text = {"frame": self.current_frame}
        actors = self.world.get_actors()
        for k,v in filter_dict.items():
            tmp = actors.filter(v)
            info_text[k] = len(tmp)
            
        return info_text
    
    def set_output_dir(self, output_dir: str):
        self.carla_config["output_dir"] = output_dir
    
    @staticmethod
    def get_agent_id(actor_id, agent_type:str='cav'):
        agent_id = IdHelper.actor_id_to_agent_id(actor_id, agent_type)
        return agent_id
    
    def get_traffic_light(self, landmark_id):
        for a_landmark_id,traffic_light in self._tls.items():
            if landmark_id == a_landmark_id:
                return traffic_light
        return None
    
    def spawn_sensors_for_agent(self, actor_id, agent_type):
        """Sapwn sensors for agent.
        todo: place rsu
        
        Parameters
        ----------
        actor_id: int
            CARLA.Actor.actor_id
        
        agent_type: str
            "cav": 1x Lidar, 1x Semantic Lidar, 4x RGB Camera, 1x GNSS
            "rsu": 1x Lidar, 1x Semantic Lidar, 2x RGB Camera, 1x GNSS
        
        Returns
        -------
        None
        """
        if agent_type == 'cav':
            agent_id = self.get_agent_id(actor_id, agent_type)
            vehicle_content = {
                "actor": self.world.get_actor(actor_id),
                "map_name": self.carla_config['world']['map_name']
            }
            config_yaml = self.carla_config["cav_sensors"]
            sensor_manager = SensorManager(
                agent_id, 
                vehicle_content, 
                self.world,
                config_yaml, 
                self.carla_config['output_dir']
            )
            self.sensor_dict[agent_id] = sensor_manager

        elif agent_type == 'rsu':
            # A RSU will be equipped with one LiDAR and two RGB Cameras

            # just spawn rsu once
            if self.has_spawned_rsu:
                return

            traffic_light = self.get_traffic_light(actor_id)
            if traffic_light is None:
                logging.warning("Found no traffic light for landmark {0}".format(actor_id))
            else:
                agent_id = self.get_agent_id(traffic_light.id, agent_type)
                self.sensor_dict[agent_id] = SensorManager(
                    agent_id,
                    {"actor": traffic_light, "map_name": self.carla_config['world']['map_name']},
                    self.world,
                    self.carla_config["rsu_sensors"],
                    self.carla_config["output_dir"]
                )
                self.used_tls[actor_id] = traffic_light.id 
        else:
            raise ValueError("`agent_type` of {0} is not supported!".format(agent_type))


    def spawn_actor(self, blueprint, transform):
        """
        Spawns a new actor.

        Parameters
        ----------
        blueprint : CARLA.Blueprint
            Blueprint of the actor to be spawned.
        
        transform : CARLA.Transform
            The transform where the actor will be spawned.
            
        Returns
        -------
        actor_id : int
            Actor's id if the actor is successfully spawned. Otherwise, INVALID_ACTOR_ID.
        """
        batch = [
            carla.command.SpawnActor(blueprint, transform).then(
                carla.command.SetSimulatePhysics(carla.command.FutureActor, False))
        ]
        response = self.client.apply_batch_sync(batch, False)[0]
        if response.error:
            logging.error('Spawn carla actor failed. %s', response.error)
            return INVALID_ACTOR_ID

        return response.actor_id
    
    def destroy_actor(self, actor_id):
        """
        Destroys the given actor and its sensors if possible.
        """
        actor = self.world.get_actor(actor_id)
        if actor is not None:
            agent_id = self.get_agent_id(actor_id)
            if agent_id in self.sensor_dict:
                self.sensor_dict[agent_id].destroy()
                self.sensor_dict.pop(agent_id)
            
            if actor.type_id.startswith("traffic"):
                self.used_tls.pop(actor_id)
                return True
            
            return actor.destroy()
        return False
    
    def tick(self, do_dumping=True):
        """
        Tick to carla simulation.
        """
        frame = self.world.tick()
        self.current_frame = frame

        # save sensor data
        if do_dumping:
            self.dump_sensor(frame)
            self.dump_map(frame)

    def dump_sensor(self, cur_timestamp):
        """
        Dump sensor data.
        """
        agent_id_list = list(self.sensor_dict.keys())
        for agent_id, sensor_manager in self.sensor_dict.items():
            sensor_manager.run_step(cur_timestamp, agent_id_list, self.actor_speed_dict)
            
    def dump_simulation_config(self, additional_info: dict):
        """
        Dump simulation config: data protocal, (traffic light) landmark id and actor id
        mapping.
        """
        if not os.path.exists(self.carla_config["output_dir"]):
            os.makedirs(self.carla_config["output_dir"])
        save_path = os.path.join(self.carla_config["output_dir"], "data_protocal.yaml")
        
        # append 
        shutil.copyfile(self.cfg_file, save_path)
        append_to_yaml({
            "traffic_lights": self.used_tls,
            "additional": additional_info
        }, 
        save_path)
        
    def dump_map(self, cur_timestamp):
        """
        Dump global info: transform,speed of all vehicles and walkers
        """
        anno_dict = generate_anno_for_map(self.world, self.actor_speed_dict)
        sub_dir = os.path.join(self.carla_config["output_dir"], "map")
        if not os.path.exists(sub_dir):
            os.makedirs(sub_dir)
        save_path = os.path.join(sub_dir, "{0:06}.yaml".format(cur_timestamp))
        save_yaml(anno_dict, save_path)

    def update_velocity(self):
        # TODO retrive each actor's speed
        return
    
    def clear(self):
        """Clear all actors"""
        current_actors = self.world.get_actors().filter('[vehicle.,walker.]*')
        
        for actor in current_actors:
            self.destroy_actor(actor.id)

        # TODO clear rsu
            
    def clear_all_actors(self):
        # clear all sensors
        all_actors = self.world.get_actors()
        current_sensors = all_actors.filter('sensor.lidar.*') +\
            all_actors.filter('sensor.camera.*') +\
            all_actors.filter('sensor.other.gnss')
        for actor in current_sensors:
            if actor.is_listening:
                actor.stop()
            actor.destroy()
        
        # clear all vehicles and walkers
        # current_actors = all_actors.filter('vehicle.*') + all_actors.filter('walker.*')
        current_actors = all_actors.filter('[vehicle.,walker.]*')
        for actor in current_actors:
            actor.destroy()

    def close(self):
        """
        Closes carla client.
        """
        if not hasattr(self, "world"):
            return

        # freeze traffic lights
        for actor in self.world.get_actors():
            if actor.type_id == 'traffic.traffic_light':
                actor.freeze(False)
            elif actor.type_id.startswith("vehicle") or actor.type_id.startswith("walker"):
                actor.destroy()

        # clear rsu sensors
        for sm in self.sensor_dict.values():
            sm.destroy()

        self.world.apply_settings(self.original_settings)
