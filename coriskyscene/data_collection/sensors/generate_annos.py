#!/usr/bin/env python
# -*- coding: utf-8 -*-
'''
@File    :   generate_annos.py
@Date    :   2023-11-09
@Author  :   Rongsong Li <rongsong.li@qq.com>
@Version :   1.0
@Desc    :   Generate annotation in OPV2V format for SUMO-CARLA co-simulation. 
'''


import os
import sys
import carla
import numpy as np
from collections import OrderedDict

from coriskyscene.data_collection.sumo_integration.constants import TYPE_ID_TO_CATEGORY,TYPE_ID_TO_EXTENT


def carla_transform_to_list(transform):
    loc = transform.location
    rot = transform.rotation
    return [loc.x, loc.y, loc.z,
            rot.roll, rot.yaw, rot.pitch]

def carla_velocity_to_scalar(velocity):
    """Vector speed to scalar speed.

    Parameters
    ----------
    velocity: CARLA.Vector3D
        Actor's velocity in m/s.

    Returns
    -------
    v: float
        Actor's scalar speed.

    Notes:
        1. To comple with SUMO, speed in vertical direction is ignored.
    """
    v = (velocity.x**2 + velocity.y**2)**0.5
    return v

def get_actor_speed(actor, actor_speed_dict):
    if actor.id in actor_speed_dict:
        return actor_speed_dict[actor.id]
    return [0.0, 0.0]  # [v_lon, v_lat]


def get_rgb_camera_info(camera_instance, lidar_instance):
    """Get camera's intrinsics, extrinsics and coordinates under
    CARLA map.
    
    Parameters
    ----------
    sensor_instance: 
        An instance of cutomized sensor.

    Returns
    -------
    info: dict
        An example is as: {
            "sensor_name":{
                "coords": [x,y,z,roll,pitch,yaw],
                "extrinsic": [[...]], # of shape 4x3, from camera to lidar
                "intrinsic": [[...]]  # of shape 3x3
            }
        }
    """
    mat_c2l = np.dot(np.array(camera_instance.sensor.get_transform().get_matrix()),
                     np.array(lidar_instance.sensor.get_transform().get_inverse_matrix()))
    mat_in = camera_instance.get_intrinsic()
    info = {
        camera_instance.name: {
            "coords": carla_transform_to_list(camera_instance.transform),
            "extrinsic": mat_c2l.tolist(),
            "intrinsic": mat_in.tolist()
        }
    }
    return info

def get_lidar_info(lidar_instance):
    return {
        "lidar_pose": carla_transform_to_list(lidar_instance.transform)
    }

def get_gnss_info(gnss_instance):
    # todo: estimate the pose from gnss measurement
    loc = gnss_instance.gnss_position
    return {
        "geo_location": [loc["lon"], loc["lat"], loc["alt"]]
    }

def get_ego_info(actor, actor_speed_dict):
    """Get ego's speed and pose. By default, only CAV nedds this.

    Parameters
    ----------
    actor: CARLA.Actor

    actor_speed_dict: dict
        Longitudinal and lateral speed of actors controlled by SUMO.
        
    Returns
    -------
    info: dict
        Ego's speed and pose
    """
    info = {
        "ego_speed": get_actor_speed(actor, actor_speed_dict),  # m/s
        "true_ego_pose": carla_transform_to_list(actor.get_transform())
    }
    return info

def get_extent_from_actor(actor):
    """In some case, carla.Actor would return extent with zero y for cycle
        and pedestrian, to avoid this, we use recorded value.
    """
    bbox = actor.bounding_box
    extent = [bbox.extent.x, bbox.extent.y, bbox.extent.z]
    # check whether all dims are bigger than zero
    eps = 0.05
    if extent[0]<eps or extent[1]<eps or extent[2]<eps:
        if actor.type_id in TYPE_ID_TO_EXTENT:
            extent = list(TYPE_ID_TO_EXTENT[actor.type_id])
        else:
            raise RuntimeError("Found no valid extent for type id '{0}': {1}".format(
                actor.type_id, extent
            ))
    return extent

def get_bbox_location_from_actor(actor):
    """
    In some cases, the actor may return bounding box with wrong
    center in y axis, e.g., [0.0, 125, 0.0] which is expected to be near
    [0, 0, 0/actor_height]. To repair this:
        1. y greater than 0.5 meters will be reset to zero.
    """
    bbox = actor.bounding_box
    thr = 0.5 # meters
    if bbox.location.y >= thr:
        return [bbox.location.x, 0.0, bbox.location.z]  # in meters
    return [bbox.location.x, bbox.location.y, bbox.location.z]  # in meters

def get_background_object_info(actor, actor_speed_dict):
    """Get background object's angle, location, 3d bbox and class.
    
    Parameters
    ----------
    actor: CARLA.Actor
        Background actor

    actor_speed_dict: dict
        Longitudinal and lateral speed of actors controlled by SUMO.
    
    Returns
    -------
    info: dict
        The said info.
    """
    transform = actor.get_transform()
    bbox = actor.bounding_box
    loc = transform.location
    rot = transform.rotation
    
    info = {}
    info["angle"] = [rot.roll, rot.yaw, rot.pitch]
    info["location"] = [loc.x, loc.y, loc.z]
    info["center"] = get_bbox_location_from_actor(actor)
    info["extent"] = get_extent_from_actor(actor)  # in meters
    info["speed"] = get_actor_speed(actor, actor_speed_dict)  # m/s
    if actor.type_id in TYPE_ID_TO_CATEGORY:
        info["category"] = TYPE_ID_TO_CATEGORY[actor.type_id].value
    elif "walker.pedestrian" in actor.type_id:
        info["category"] = TYPE_ID_TO_CATEGORY["walker.pedestrian"].value
    else:
        info["category"] = TYPE_ID_TO_CATEGORY["other"].value

    # new added!!!
    info["type_id"] = actor.type_id
    return info


def filter_objects_by_rectangle(objects, ego_transform, range_x=[-140, 140], range_y=[-40, 40]):
    """Filter objects using rectangle whose x-axis is along the front/rear direction of a agent
    and y-axis along the left/right direction.

    Parameters
    ----------
    objects: List[CARLA.Actor]
        Actors in carla.

    ego_transfrom: CARLA.Transform
        center of the recrangle

    range_x: List
        Evaluation range in x direction

    range_y: List
        Evaluation range in y direction

    Returns
    -------
    filtered_objects: List[CARLA.Actor]

    """
    bbox = carla.BoundingBox(
        carla.Location((range_x[0]+range_x[1])/2.0, 
                       (range_y[0]+range_y[1])/2.0, 0),
        carla.Vector3D((range_x[1]-range_x[0])/2.0, 
                       (range_y[1]-range_y[0])/2.0, 1)
    )
    filtered_objects = []
    for obj in objects:
        loc = obj.get_transform().location
        # loc = carla.Location(loc.x, loc.y, 0)
        if bbox.contains(loc, ego_transform):
            filtered_objects.append(obj)
    return filtered_objects

def filter_objects_by_circle(objects, ego_transform, radius=120.0):
    """Filter objects using circle centered at ego agent.

    Parameters
    ----------
    objects: List[CARLA.Actor]
        Actors in carla

    ego_transform: CARLA.Transform
        Center of the circle.

    radius: float
        Evaluation range

    Returns
    -------
    filtered_objects: List[CARLA.Actor]
        Filtered actors in carla.
    """
    filtered_objects = [obj for obj in objects\
                        if obj.get_transform().location.distance(
                            ego_transform.location
                        )<radius]
    return filtered_objects

def generate_anno_for_agent(world, actor_id, sensor_manager, actor_list, actor_speed_dict):
    """Generate annotation info for a connected agent (i.e., CAV and RSU).
    By default, we consider the following 5 classes: vehicles, trucks, bikes, 
    mortors and pedestrians, which should lie in 

    Parameters
    ----------
    world: CARLA.world

    ego_transform: CARLA.Transform
        Center to filter objects.

    sensor_manager: SensorManager
        Sensor manager of the agent

    agent_list: List[CARLA.Actor]
        All carla actors that can communicate.

    actor_speed_dict: dict
        Longitudinal and lateral speed of actors controlled by SUMO. Format:
        {"actor_id": [v_lon, v_lat]}
    
    Returns
    -------
    anno_dict: dict
        The said annotation info.

    Notes
    -----
    1. By default, a sensor must be with a lidar pose
    """
    anno_dict = OrderedDict()
    
    # info of sensor: lidar, camera, gnss
    lidar_instance = None
    for sensor_instance in sensor_manager.sensor_list:
        if "lidar" == sensor_instance.name:
            lidar_instance = sensor_instance
            break
    for sensor_instance in sensor_manager.sensor_list:
        if "lidar" == sensor_instance.name:
            anno_dict.update(get_lidar_info(sensor_instance))
        elif "camera" in sensor_instance.name:
            anno_dict.update(get_rgb_camera_info(sensor_instance, lidar_instance))
        elif "gnss" in sensor_instance.name:
            anno_dict.update(get_gnss_info(sensor_instance))
        else:
            pass

    # info of ego
    ego = world.get_actor(actor_id)
    if ego is None:
        raise RuntimeError("Found no actor for actor_id `{0}` in carla".format(actor_id))
    anno_dict.update(get_ego_info(ego, actor_speed_dict))

    # info of connected agents
    COMM_RANGE = 70
    actor_list = filter_objects_by_circle(actor_list, ego.get_transform(), COMM_RANGE)
    conn_actor_id_list = [actor.id for actor in actor_list]
    anno_dict["conn_agents"] = conn_actor_id_list

    # info of record 3d bboxes
    ego_transform = ego.get_transform()
    if ego.type_id.startswith("traffic"):
        ego_transform.rotation.yaw += 90  # for rsu, x-axis along road
    
    anno_dict["objects"] = {}
    objects = world.get_actors().filter("[vehicle.,walker.]*")
    objects = filter_objects_by_rectangle(objects, ego_transform)
    for obj in objects:
        if obj.id == actor_id:  # ignore ego
            continue
        anno_dict["objects"][obj.id] = get_background_object_info(obj, actor_speed_dict)

    return anno_dict

def generate_anno_for_map(world, actor_speed_dict):
    """Generate annotations of all the objects in the map. An object's communication
    ability, category, pose, speed.

    Parameters
    ----------
    world: CARLA.World
        The carla world

    actor_list: List[CARLA.Actor]
        All actors that can communicate with others.

    actor_speed_dict: dict
        Actors' longtitudinal and lateral speed.
    
    Returns
    -------
    anno_dict: dict

    """
    anno_dict = OrderedDict()
    anno_dict["objects"] = {}

    # objects info
    objects = world.get_actors().filter("[vehicle.,walker.]*")
    for obj in objects:
        anno_dict["objects"][obj.id] = get_background_object_info(obj, actor_speed_dict)
    return anno_dict
    
