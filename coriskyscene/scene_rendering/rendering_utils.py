#!/usr/bin/env python
# -*- coding: utf-8 -*-
'''
@File    :   rendering_utils.py
@Date    :   2024-01-24
@Author  :   Rongsong Li <rongsong.li@qq.com>
@Version :   1.0
@Desc    :   Utility for rendering traffic scene
'''

import os
import json
import itertools
import numpy as np

import carla


# ==================================================================
# - CAV and RSU for logical scenarios
# ==================================================================
def get_cav_ids_for_logical_scenario(logical_scenario_name:str, scene):
    """
    Get cav ids for certain logical scenario.
    Possible CAV: ego, occluder, occludee, and npc.

    """
    valid_cav_ids = list(itertools.chain(*[el[2] for el in scene.occlusion_pairs]))
    valid_cav_ids.append(scene.ego_id)

    if logical_scenario_name == "ruralcurve_l2" or logical_scenario_name == "parkingexit":
        # take occludee as CAV if it's car
        occludee_id = scene.occlusion_pairs[0][1]
        for scene_actor in scene.actor_list:
            if scene_actor.id == occludee_id and scene_actor.vclass == 'car':
                valid_cav_ids.append(occludee_id)
                break
    return valid_cav_ids

def get_traffic_lights_for_logical_scenario(logical_scenario_name: str):
    """
    One can retrive "coriskyscene/data_collection/sensors/sensor_position.py"
    to find out traffic lights for target junction.

    Parameters
    ----------
    logical_scenario_name : str

    Returns
    -------
    tls : list
        A list of traffic landmark id of traffic lights.
    
    Notes
    -----
    1. Specify related traffic lights as RSU here for your customized scenes.
    """
    tls = []
    if logical_scenario_name == "juncrush":
        # in X junction "1574"
        tls = [2405, 2406, 2407, 2408]
        tls = [str(el) for el in tls]
    elif logical_scenario_name in ["juncreverse", "juncsame", "juncturn"]:
        # in X junction 
        tls = [2389, 2390, 2391, 2392]
        tls = [str(el) for el in tls]

    return tls

# ==================================================================
# - Layered Map
# ==================================================================
def get_layered_map_name(map_name:str):
    new_name = map_name + "_Opt"
    return new_name


# ==================================================================
# - Coordinate system conversion
# ==================================================================
def opendrive_to_carla_transform(x, y, z, h, p=0, r=0):
    """Convert object's pose from OpenDRIVE (right-hand) 
    into Transform in CARLA (left-hand).
    
    Paramters
    ---------
    x,y,z : float
        The global x coordinate in meters.

    h,p,r : float
        The global heading/pitch/roll angle in radians

    Returns
    -------
    transform : CARLA.Transform

    Refs
    ----
    1. https://github.com/carla-simulator/scenario_runner/blob/master/srunner/tools/openscenario_parser.py#L629
    """
    h = np.rad2deg(h)
    p = np.rad2deg(p)
    r = np.rad2deg(r)
    transform = carla.Transform(
        carla.Location(x, -y, z),
        carla.Rotation(yaw=-h, pitch=p, roll=r)
    )
    return transform


# ============================================================
# - Operations about scenario and scene
# ============================================================
def get_logical_scenario_name_from_path(path:str):
    """Infer logical scenario name from path of scenes.
    
    Parameters
    ----------
    path : str
        E.g., "overtake.json"

    Returns
    -------
    logical_scenario_name : str
        E.g., "overtake"
    """
    fname,_ = os.path.splitext(os.path.basename(path))
    logical_scenario_name = fname
    return logical_scenario_name

def get_logical_scenario_name_from_scene_id(scene_id: str):
    """Infer logical scenario name from scene id.
    
    Parameters
    ----------
    scene_id : str
        E.g., "overtake_00001".

    Returns
    -------
    name : str
        E.g., "overtake"
    """
    return scene_id.split("_")[0]

def construct_scene_desc(scene, id_mapping):
    """Construct traffic scene description."""
    # convert actor id to carla actor id.
    scene_desc = {
        "id": scene.id,
        "town_name": scene.town_name,
        "ego_id": scene.ego_id,
        "actor_list": [scene_actor.id for scene_actor in scene.actor_list],
        "occlusion_pairs": scene.occlusion_pairs,
        "id_mapping": id_mapping
    }
    return scene_desc