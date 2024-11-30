#!/usr/bin/env python
# -*- coding: utf-8 -*-
'''
@File    :   render_scene.py
@Date    :   2024-01-10
@Author  :   Rongsong Li <rongsong.li@qq.com>
@Version :   1.0
@Desc    :   Render the generated scenes
'''

import os
import os
os.environ["NUMEXPR_MAX_THREADS"] = "16"
import time
import argparse
import logging

from coriskyscene.scene_generation import traffic_scene as trasc
from coriskyscene.scene_rendering.carla_simulation import CarlaSimulation, INVALID_ACTOR_ID
from coriskyscene.common_utils import logging_utils
from coriskyscene.scene_rendering import rendering_utils



# global variable
# should be rewrite as logical scenario's name
LOGGER_NAME = "root"

def parse_args():
    argparser = argparse.ArgumentParser(description=__doc__)
    argparser.add_argument('--scene-path', type=str, help='path of traffic scenes')
    argparser.add_argument('--carla-cfg-file', type=str, help='carla configuration file')
    argparser.add_argument('--carla-host',
                           metavar='H',
                           default='127.0.0.1',
                           help='IP of the carla host server (default: 127.0.0.1)')
    argparser.add_argument('--carla-port',
                           metavar='P',
                           default=2000,
                           type=int,
                           help='TCP port to listen to (default: 2000)')
    argparser.add_argument('--step-length',
                           default=0.05,
                           type=float,
                           help='set fixed delta seconds (default: 0.05s)')
    argparser.add_argument('--gather-data',
                           action='store_true',
                           help='If set all cars will be equipped with sensors and collecting data.'\
                            'Otherwise just run simulation. (default: False)')
    argparser.add_argument('--debug', action='store_true', help='enable debug messages')
    arguments = argparser.parse_args()

    if arguments.debug:
        logging.basicConfig(format='%(levelname)s: %(message)s', level=logging.DEBUG)
    else:
        logging.basicConfig(format='%(levelname)s: %(message)s', level=logging.INFO)

    return arguments


def spawn_actors_in_scene(carla_simulation, scene, logical_scenario_name, bp_lib=None):
    """Spawn actor in a scene
    
    Parameters
    ----------
    carla_simulation : 

    scene : TrafficScene

    logical_scenario_name : str

    bp_lib : 
        CARLA's blueprint library.

    Returns
    -------
    id_mapping : dict
        The mapping from scene_actor_id to carla_actor_id. If the scene actor
        is static obstacle, its carla id would be -1.
    """
    MAX_TRY = 10
    DELTA_Z = 0.02  # meters
    sidewalk_height = 0.2 # meters

    # get logger
    logger = logging.getLogger(LOGGER_NAME)

    # get blueprint library
    if bp_lib is None:
        bp_lib = carla_simulation.world.get_blueprint_library()

    id_mapping = {}  # {scene_actor_id: carla_actor_id}
    
    # go through all actors
    for scene_actor in scene.actor_list:
        # escape the virtual object
        if scene_actor.type_id.startswith("virtual"):
            continue
        blueprint = bp_lib.find(scene_actor.type_id)
        if not blueprint:
            logger.warning("Cannot find blueprint for type_id '{0}'".format(scene_actor.type_id))
            return {}

        x,y,z = scene_actor.waypoint.coord
        # hardcode sidewalk's height
        z += sidewalk_height if scene_actor.waypoint.lane_type == 'sidewalk' else 0.0
        
        transform = rendering_utils.opendrive_to_carla_transform(
            x, y , z, scene_actor.waypoint.heading
        )
        transform.location.z += 0.04
        
        has_tried = 0
        while has_tried<MAX_TRY:
            # increase z to avoid collision with road surface
            transform.location.z += has_tried*DELTA_Z
            actor_id = carla_simulation.spawn_actor(blueprint, transform)
            if actor_id != INVALID_ACTOR_ID:
                break
            has_tried += 1

        if actor_id == INVALID_ACTOR_ID:
            # fail to spawn
            logger.warning("Fail to spawn scene actor {0} at {1}".format(scene_actor.id, transform))
            return {}
        id_mapping[scene_actor.id] = actor_id
        
    # Spawn sensors for ego and other cav
    valid_cav_ids = rendering_utils.get_cav_ids_for_logical_scenario(logical_scenario_name, scene)
    for scene_actor in scene.actor_list:
        if scene_actor.id in valid_cav_ids:
            carla_simulation.spawn_sensors_for_agent(id_mapping[scene_actor.id], "cav")

    # Spawn sensors for rsu
    if not carla_simulation.has_spawned_rsu:
        tls = rendering_utils.get_traffic_lights_for_logical_scenario(logical_scenario_name)
        for tl in tls:
            carla_simulation.spawn_sensors_for_agent(tl, "rsu")
        carla_simulation.has_spawned_rsu = True
    else:
        # update output root
        for agent_id,sm in carla_simulation.sensor_dict.items():
            if agent_id.startswith("rsu"):
                sm.reset_output_root_to(carla_simulation.carla_config["output_dir"])
    
    return id_mapping

def run(args):
    # load traffic scenes from one logical scenario, so
    # these scenes share the same map
    scene_list = trasc.load_traffic_scenes(args.scene_path)
    map_name = scene_list[0].town_name
    map_name = rendering_utils.get_layered_map_name(map_name)
    logical_scenario_name = rendering_utils.get_logical_scenario_name_from_path(args.scene_path)

    # initialize CARLA
    carla_simulation = CarlaSimulation(
        args.carla_cfg_file, 
        args.carla_host, 
        args.carla_port,
        args.step_length,
        map_name
    )
    root = os.path.join(carla_simulation.carla_config["output_dir"],
                        logical_scenario_name)
    if os.path.exists(root):
        logging.warning("The output directory already exists!")
        return
    
    # initialzie logger
    global LOGGER_NAME
    LOGGER_NAME = logical_scenario_name
    logger = logging_utils.init_logger(
        logical_scenario_name,
        os.path.join(os.path.dirname(os.path.abspath(__file__)), "logs", logical_scenario_name+".log"),
        logging.DEBUG if args.debug else logging.INFO
    )
    logger.info("Start processing logical scenario: {0}".format(logical_scenario_name))


    # render each traffic scene
    num_rendered_scenes = 0
    start_index = 0
    end_index = len(scene_list)
    # end_index = 2
    relaunch_carla_every = 200

    start_time = time.time()
    for i,scene in enumerate(scene_list):
        if i<start_index:
            continue
        if i>=end_index:
            break

        # launch carla
        if num_rendered_scenes%relaunch_carla_every == 0:
            logger.info("Start (re)launching CARLA...")
            carla_simulation.relaunch_carla_server()
            if not carla_simulation.init_carla():
                logger.error("Fail to launch CARLA! Exit Rendering.")
                break
            else:
                logger.info("Launch CARLA successfully!")
                bp_lib = carla_simulation.world.get_blueprint_library()

        logger.info("Processing {0}th scene '{1}'".format(i, scene.id))
        
        # set output dir
        carla_simulation.set_output_dir(
            os.path.join(root, scene.id)
        )
        
        id_mapping = spawn_actors_in_scene(carla_simulation, scene, logical_scenario_name, bp_lib)
        print("id_mapping:", id_mapping)
        if len(id_mapping)==0:
            logger.error("Fail to spawn actors for scene '{0}'".format(scene.id))
        else:
            # dump data
            carla_simulation.tick(do_dumping=True)
            carla_simulation.dump_simulation_config(
                {"scene_desc": rendering_utils.construct_scene_desc(scene, id_mapping)}
            )
            num_rendered_scenes += 1
        
        logger.debug(carla_simulation.get_current_overview())
        time.sleep(0.5)  # wait for server to complete rendering
        
        carla_simulation.clear()
        carla_simulation.tick(do_dumping=False)
        time.sleep(0.5)  # wait for server to complete clearing

        # check whether all actors are cleared
        logger.info("after clear:"+str(carla_simulation.get_current_overview()))

    end_time = time.time()
    carla_simulation.close()
    carla_simulation.kill_carla_server()

    logger.info("start index: {0}, end index: {1}".format(start_index, end_index))
    logger.info("{0} scenes are rendered, comsuming: {1}s".format(num_rendered_scenes, end_time-start_time))



if __name__ == '__main__':
    run(parse_args())