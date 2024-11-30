#!/usr/bin/env python

# Copyright (c) 2020 Computer Vision Center (CVC) at the Universitat Autonoma de
# Barcelona (UAB).
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.
#
# Modified by: Rongsong Li <rongsong.li@qq.com>
""" This module is responsible for the management of the carla simulation. """

# ==================================================================================================
# -- imports ---------------------------------------------------------------------------------------
# ==================================================================================================

import os
import logging
import time
import shutil
import carla  # pylint: disable=import-error

from coriskyscene.data_collection.sumo_integration.constants import \
    INVALID_ACTOR_ID, SPAWN_OFFSET_Z, CustomActorType, RSU_LANDMARK_IDS
from coriskyscene.data_collection.sensors.sensor_manager import SensorManager
from coriskyscene.data_collection.sensors.generate_annos import generate_anno_for_map
from coriskyscene.data_collection.util.id_helper import IdHelper
from coriskyscene.common_utils.yaml_util import load_yaml, save_yaml, append_to_yaml
from coriskyscene.common_utils.misc import bcolors


# ==================================================================================================
# -- carla simulation ------------------------------------------------------------------------------
# ==================================================================================================


class CarlaSimulation(object):
    """
    CarlaSimulation is responsible for the management of the carla simulation.
    """
    def __init__(self, cfg_file, host, port, step_length, extra_cfg:dict={}):
        self.cfg_file = cfg_file
        self.carla_config = load_yaml(cfg_file)
        self.client = carla.Client(host, port)
        self.client.set_timeout(180.0)

        # load world
        map_name = self.carla_config['world']['map_name']
        try:
            world = self.client.load_world(map_name)
        except RuntimeError:
            print(f"{bcolors.FAIL} %s is not found in your CARLA repo!{bcolors.ENDC}"%map_name)
        
        # Configuring carla simulation in sync mode
        self.original_settings = world.get_settings()
        new_settings = world.get_settings()
        new_settings.synchronous_mode = self.carla_config['world']['sync_mode']
        new_settings.fixed_delta_seconds = step_length
        world.apply_settings(new_settings)
        self.world = world

        traffic_manager = self.client.get_trafficmanager()
        traffic_manager.set_synchronous_mode(True)

        self.blueprint_library = self.world.get_blueprint_library()
        self.step_length = step_length
        default_extra_cfg = {
            "start_frame": 0,  # frame that we start recording data
            "end_frame": 1e6,  # frame that we end recording data
            "save_every": 2    # data is saved every ``save_every`` frame
        }
        default_extra_cfg.update(extra_cfg)
        self.extra_cfg = default_extra_cfg

        self.current_frame = 0

        # output dir
        scenario_name = map_name + "__" + time.strftime("%Y_%m_%d_%H_%M_%S", time.localtime())
        self.carla_config["output_dir"] = os.path.join(self.carla_config["output_dir"],
                                                       scenario_name)

        # The following sets contain updated information for the current frame.
        self._active_actors = set()
        self.spawned_actors = set()
        self.destroyed_actors = set()

        # speed of actors controlled by SUMO
        self.actor_speed_dict = {}  # {actor_id: [v_lon, v_lat]}

        # Set traffic lights.
        self._tls = {}  # {landmark_id: traffic_ligth_actor}
        self._has_spawned_rsu = False  # can be modified only once!!!

        tmp_map = self.world.get_map()
        for landmark in tmp_map.get_all_landmarks_of_type('1000001'):
            if landmark.id != '':
                traffic_light = self.world.get_traffic_light(landmark)
                if traffic_light is None:
                    logging.warning('Landmark %s is not linked to any traffic light', landmark.id)
                    continue
                self._tls[landmark.id] = traffic_light
                traffic_light.set_state(carla.TrafficLightState.Green)

        # sensors
        self.sensor_dict = dict()  # {"agent_id": {"sensor_manager": xx, }}

    def get_actor(self, actor_id):
        """
        Accessor for carla actor.
        """
        return self.world.get_actor(actor_id)
    
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
    
    @staticmethod
    def get_agent_id(actor_id, agent_type:str='cav'):
        agent_id = IdHelper.actor_id_to_agent_id(actor_id, agent_type)
        return agent_id
    
    @staticmethod
    def get_custom_actor_type(actor):
        """Returns custom type of the given carla actor."""
        if actor.type_id.startswith("walker"):
            return CustomActorType.WALKER
        elif actor.type_id.startswith("vehicle"):
            return CustomActorType.VEHICLE
        else:
            return CustomActorType.UNKNOWN

    # This is a workaround to fix synchronization issues when other carla clients remove an actor in
    # carla without waiting for tick (e.g., running sumo co-simulation and manual control at the
    # same time)
    def get_actor_light_state(self, actor_id):
        """
        Accessor for carla actor light state.

        If the actor is not alive, returns None.
        """
        try:
            actor = self.get_actor(actor_id)
            if actor.type_id.startswith("walker"):  # walker has no light
                return None
            return actor.get_light_state()
        except RuntimeError:
            return None

    @property
    def traffic_light_ids(self):
        return set(self._tls.keys())

    def get_traffic_light_state(self, landmark_id):
        """
        Accessor for traffic light state.

        If the traffic ligth does not exist, returns None.
        """
        if landmark_id not in self._tls:
            return None
        return self._tls[landmark_id].state

    def switch_off_traffic_lights(self):
        """
        Switch off all traffic lights.
        """
        for actor in self.world.get_actors():
            if actor.type_id == 'traffic.traffic_light':
                actor.freeze(True)
                # We set the traffic light to 'green' because 'off' state sets the traffic light to
                # 'red'.
                actor.set_state(carla.TrafficLightState.Green)

    @staticmethod
    def get_tl_landmarks(world):
        """Retrive all landmarks of traffic lights. Only one in a group will be kept."""
        landmark_list =  world.get_map().get_all_landmarks_of_type('1000001')
        landmark_ids = set()
        nodup_landmark_list = []
        for landmark in landmark_list:
            if landmark.id != "" and landmark.id not in landmark_ids:
                landmark_ids.add(landmark.id)
                nodup_landmark_list.append(landmark)
        return nodup_landmark_list

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
                "actor": self.get_actor(actor_id),
                "map_name": self.carla_config['world']['map_name']
            }
            config_yaml = self.carla_config["cav_sensors"]
            sensor_manager = SensorManager(agent_id, vehicle_content, self.world,
                                           config_yaml, self.carla_config['output_dir'])
            self.sensor_dict[agent_id] = sensor_manager

        elif agent_type == 'rsu':
            # we take each traffic light as a RSU
            # A RSU will be equipped with one LiDAR and two RGB Cameras
            if self._has_spawned_rsu:
                return
            self._has_spawned_rsu = True
            count = 0
            tl_info = {}
            for landmark_id,traffic_light in self._tls.items():
                map_name = self.carla_config['world']['map_name']
                if map_name not in RSU_LANDMARK_IDS:
                    logging.warning("No specified RSU in map {0}!".format(map_name))
                    return
                if landmark_id in RSU_LANDMARK_IDS[map_name]:
                    agent_id = self.get_agent_id(traffic_light.id, agent_type)
                    self.sensor_dict[agent_id] = SensorManager(
                        agent_id,
                        {"actor": traffic_light, "map_name": map_name},
                        self.world,
                        self.carla_config["rsu_sensors"],
                        self.carla_config["output_dir"]
                    )
                    count += 1
                    tl_info.update({landmark_id: traffic_light.id})
            logging.info("[carla_simulation] Spawn sensors for {0} road side units successfully!".format(count))
            # logging.debug("[carla_simulation] tl info: {0}".format(tl_info))

            # save simulation config
            self.simulation_config_dumping()
        else:
            raise ValueError("`agent_type` of {0} is not supported!".format(agent_type))


    def spawn_actor(self, blueprint, transform):
        """
        Spawns a new actor.

            :param blueprint: blueprint of the actor to be spawned.
            :param transform: transform where the actor will be spawned.
            :return: actor id if the actor is successfully spawned. Otherwise, INVALID_ACTOR_ID.
        """
        transform = carla.Transform(transform.location + carla.Location(0, 0, SPAWN_OFFSET_Z),
                                    transform.rotation)

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
                return True
            return actor.destroy()
        return False

    def synchronize_actor(self, actor_id, transform, lights=None):
        """
        Updates vehicle/person state.

            :param vehicle_id: id of the actor to be updated.
            :param transform: new vehicle transform (i.e., position and rotation).
            :param lights: new vehicle light state.
            :param velocity: new vehicle velocity but just in x,y direction.
            :return: True if successfully updated. Otherwise, False.
        """
        actor = self.world.get_actor(actor_id)
        if actor is None:
            return False

        actor.set_transform(transform)
        if lights is not None and actor.type_id.startswith("vehicle"):
            actor.set_light_state(carla.VehicleLightState(lights))
    
        return True
                
    def synchronize_traffic_light(self, landmark_id, state):
        """
        Updates traffic light state.

            :param landmark_id: id of the landmark to be updated.
            :param state: new traffic light state.
            :return: True if successfully updated. Otherwise, False.
        """
        if not landmark_id in self._tls:
            logging.warning('Landmark %s not found in carla', landmark_id)
            return False

        traffic_light = self._tls[landmark_id]
        traffic_light.set_state(state)
        return True
    
    def synchronize_velocity(self, actor_speed_dict):
        """Updates actor speed.
        This is workaround when actors are managed by SUMO. In such case,
        we can't call `actor.set_target_velocity` because carla will update 
        actor's location and rotation based on this.
        
        Parameters
        ----------
        actor_speed_dict: dict
            Each actor's longitudinal and lateral speed from SUMO. An example
            is {1011: [0.33, 0.10]}, in which 1011 is actor_id.

        Returns
        -------
        None
        """
        self.actor_speed_dict = actor_speed_dict

    def tick(self):
        """
        Tick to carla simulation.
        """
        frame = self.world.tick()
        self.current_frame = frame

        # Update data structures for the current frame.
        all_actors = self.world.get_actors()
        current_actors = set([actor.id for actor in all_actors.filter("vehicle.*")]) |\
            set([actor.id for actor in all_actors.filter("walker.*")])
        self.spawned_actors = current_actors.difference(self._active_actors)
        self.destroyed_actors = self._active_actors.difference(current_actors)
        self._active_actors = current_actors

        # save sensor data
        if frame>=self.extra_cfg['start_frame'] and frame<=self.extra_cfg['end_frame']\
            and frame%self.extra_cfg['save_every'] == 0:
            self.sensor_dumping(frame)
            self.map_dumping(frame)

    def sensor_dumping(self, cur_timestamp):
        """
        Dump sensor data.
        """
        agent_id_list = list(self.sensor_dict.keys())
        for agent_id, sensor_manager in self.sensor_dict.items():
            sensor_manager.run_step(cur_timestamp, agent_id_list, self.actor_speed_dict)
            
    def map_dumping(self, cur_timestamp):
        """
        Dump global info: transform,speed of all vehicles and walkers
        """
        anno_dict = generate_anno_for_map(self.world, self.actor_speed_dict)
        sub_dir = os.path.join(self.carla_config["output_dir"], "map")
        if not os.path.exists(sub_dir):
            os.makedirs(sub_dir)
        save_path = os.path.join(sub_dir, "{0:06}.yaml".format(cur_timestamp))
        save_yaml(anno_dict, save_path)

    def simulation_config_dumping(self):
        """
        Dump simulation config: data protocal, (traffic light) landmark id and actor id
        mapping.
        """
        if not os.path.exists(self.carla_config["output_dir"]):
            os.makedirs(self.carla_config["output_dir"])
        save_path = os.path.join(self.carla_config["output_dir"], "data_protocal.yaml")
        map_name = self.carla_config['world']['map_name']
        used_tls = {}  # {"landmark_id": "actor_id"}
        
        if map_name in RSU_LANDMARK_IDS:
            for landmark_id,tl in self._tls.items():
                if landmark_id in RSU_LANDMARK_IDS[map_name]:
                    used_tls[landmark_id] = tl.id

        # append 
        shutil.copyfile(self.cfg_file, save_path)
        append_to_yaml({"traffic_lights": used_tls}, save_path)

    
    def close(self):
        """
        Closes carla client.
        """
        # freeze traffic lights
        for actor in self.world.get_actors():
            if actor.type_id == 'traffic.traffic_light':
                actor.freeze(False)

        # clear rsu sensors
        for sm in self.sensor_dict.values():
            sm.destroy()

        self.world.apply_settings(self.original_settings)
