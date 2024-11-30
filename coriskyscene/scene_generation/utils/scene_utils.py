#!/usr/bin/env python
# -*- coding: utf-8 -*-
'''
@File    :   scene_utils.py
@Date    :   2024-01-14
@Author  :   Rongsong Li <rongsong.li@qq.com>
@Version :   1.0
@Desc    :   some utilities for scene
'''

import logging
import numpy as np

from coriskyscene.scene_generation.utils import route_utils


# =========================================================
# - Sample waypoints for vehicles in route
# =========================================================
def sample_waypoint_along_lane(map_object, road_id, lane_id, s, ds, direction="forward"):
    """Sample a waypoint along the given route
    
    Parameters
    ----------
    map_object : 

    road_id : int

    lane_id : int

    s : float

    ds : float

    route : list
        e.g.: [[connectingRoadId, outgoingLaneId, contactPoint]]


    Returns
    -------
    wp : Waypoint | None
        Returns None if target s_pos is out of range of the road.
    
    Refs
    ----
    1. Multiple laneSections are not considered.
    """
    assert s>0.0
    assert ds>=0.0
    assert direction in ['forward', 'backward']

    road = map_object.getRoad(road_id)

    # case 1: current lane is right lane
    if lane_id < 0:
        if direction == "forward":
            target_s = s + ds
        elif direction == "backward":
            target_s = s - ds

    # case 2: current lane is left lane
    elif lane_id > 0:
        if direction == 'forward':
            target_s = s - ds
        elif direction == 'backward':
            target_s = s + ds
    
    # center line is forbidden
    else:
        raise ValueError("center line is forbidden!")
    
    if target_s<0 or target_s > road.length:
        return None
    
    wp = road.calcWaypoint(0, lane_id, target_s)
    return wp

def sample_leading_vehicle_for_actor(map_object, route, vehicle_sampler, actor, route_len_list=None, num_tries:int=1):
    """Sample a leading vehicle along route for the given actor. 

    Parameters
    ----------
    map_object :

    route :

    vehicle_sampler : 

    actor : CarlaActor
        If None, we sample from route's start.

    route_len_list : 

    num_tries : int
        we will attempt to sample vehicle up to `num_tries` times.
    
    Returns
    -------
    vehicle : dict
        Returns None if actor or vehicle out of the route. Normal return: 
        {"type_id": xxx, "waypoint": xxx}
    """
    # calculate the actor's location in route
    if actor is None:
        actor_front_route_s = 0.0
        actor_type_id = None
    else:
        actor_front_route_s = route_utils.calc_route_s_for_actor(map_object, route, actor, "front")
        actor_type_id = actor.type_id
    # the actor is out of the route
    if actor_front_route_s is None:
        return None
    
    if route_len_list is None:
        route_len_list = route_utils.get_route_len_list(map_object, route)

    for i in range(num_tries):
        # sample leading vehicle and its location
        vehicle_type_id,vehicle_half_length,spacing = vehicle_sampler.sample_leading_vehicle(actor_type_id)
        # calculate location of lv center
        vehicle_center_route_s = actor_front_route_s + spacing + vehicle_half_length

        # lv is out of the route
        if vehicle_center_route_s >= route_len_list[-1]:
            continue
        
        wp = route_utils.calc_waypoint_from_route_s(map_object, route, vehicle_center_route_s, route_len_list)
        vehicle = {
            "type_id": vehicle_type_id,
            "waypoint": wp
        }
        return vehicle
    
    return None

def sample_following_vehicle_for_actor(map_object, route, vehicle_sampler, actor, route_len_list=None, num_tries:int=1):
    """Sample a following vehicle against route for the given actor.
    
    """
    # calculate the actor's location in route
    if actor is None:
        actor_rear_route_s = 0.0
        actor_type_id = None
    else:
        actor_rear_route_s = route_utils.calc_route_s_for_actor(map_object, route, actor, "rear")
        actor_type_id = actor.type_id
    # the actor is out of the route
    if actor_rear_route_s is None:
        return None
    
    if route_len_list is None:
        route_len_list = route_utils.get_route_len_list(map_object, route)

    for i in range(num_tries):
        # sample following vehicle and its locaiton
        vehicle_type_id,vehicle_half_length,spacing = vehicle_sampler.sample_leading_vehicle(actor_type_id)
        # calculate location of lv center
        vehicle_center_route_s = actor_rear_route_s - spacing - vehicle_half_length

        # lv is out of the route
        if vehicle_center_route_s < 0:
            continue
        
        wp = route_utils.calc_waypoint_from_route_s(map_object, route, vehicle_center_route_s, route_len_list)
        vehicle = {
            "type_id": vehicle_type_id,
            "waypoint": wp
        }
        return vehicle
    
    return None


def sample_vehicles_along_route(map_object, init_s, route, vehicle_sampler, max_range=125.0, init_vehicle_type_id=None):
    """Following the direction of the lane and sample the leading vehicles auto regressively.
    TODO add support for multiple laneSections

    Parameters
    ----------
    map_object : OpenDRIVE Map

    road_id :

    lane_id :

    init_s : float
        The s value of front bumper in meters from route's start to current location.

    route : list
        a series of lanes in order of normal dirving

    vehicle_sampler :
        The sampler returns (vehicle type, vehicle half length, vehicle spacing) everytime.

    max_range : float
        The sampling range in meters along s-axis

    Returns
    -------
    spawned_vehicle_list : list
        E.g.: [
            {"type_id": "vehicle.audi.a2", "waypoint": waypoint for the vehicle}
        ]
    """
    assert init_s>=0.0

    current_route_s = init_s  # location of front bumper of current vehicle from the start of the route
    total_ds = 0.0      # total length along s-axis that have been visited
    spawned_vehicle_list = []
    following_vehicle_type_id = init_vehicle_type_id
    count = 0

    # percompute to save time
    route_len_list = route_utils.get_route_len_list(map_object, route)
    
    # sample vehicles atuo regressively
    while True:
        # sample leading vehicle and its location
        vehicle_type_id,vehicle_half_length,spacing = vehicle_sampler.sample_leading_vehicle(following_vehicle_type_id)
        following_vehicle_type_id = vehicle_type_id

        logging.debug("{0}-th sampling: vehicle_type: {1}, spacing: {2}".format(count, vehicle_type_id, spacing))
        count += 1

        # calculate the location of vehicle center
        vehicle_center_route_s = current_route_s + spacing + vehicle_half_length
        total_ds += spacing + vehicle_half_length*2.0
        
        # out range of the route or maximum range
        if vehicle_center_route_s >= route_len_list[-1] or total_ds > max_range:
            break

        wp = route_utils.calc_waypoint_from_route_s(map_object, route, vehicle_center_route_s, route_len_list)

        # record new vehicles
        spawned_vehicle_list.append({
            "type_id": vehicle_type_id,
            "waypoint": wp
        })

        # update
        current_route_s = vehicle_center_route_s + vehicle_half_length

    return spawned_vehicle_list


def sample_vehicles_against_route(map_object, init_s, route, vehicle_sampler, max_range=125.0, init_vehicle_type_id=None):
    """sample vehicles against the direction of the route auto regressively.
    TODO add support for multiple laneSections

    Parameters
    ----------
    map_object : OpenDRIVE Map

    road_id :

    lane_id :

    init_s :
        The s value of rear bumper in meters from route's start to current location.
    
    route : list
        lanes in order of normal dirving

    vehicle_sampler :
        The sampler returns (vehicle type, vehicle half length, vehicle spacing) everytime.

    max_range : float
        The sampling range in meters along s-axis

    Returns
    -------
    spawned_vehicle_list : list
        E.g.: [
            {"type_id": "vehicle.audi.a2", "waypoint": waypoint for the vehicle}
        ]
    """
    assert init_s>=0.0

    current_route_s = init_s  # location of rear bumper of current vehicle from start of the route
    total_ds = 0.0 
    spawned_vehicle_list = []
    leading_vehicle_type_id = init_vehicle_type_id
    count = 0

    route_len_list = route_utils.get_route_len_list(map_object, route)

    # sample vehicles auto regressively
    while True:
        # sample following vehicle and its location
        vehicle_type_id,vehicle_half_length,spacing = vehicle_sampler.sample_following_vehicle(leading_vehicle_type_id)
        leading_vehicle_type_id = vehicle_type_id

        logging.debug("{0}-th sampling: vehicle_type: {1}, spacing: {2}".format(count, vehicle_type_id, spacing))
        count += 1

        # calculate the location of vehicle center
        vehicle_center_route_s = current_route_s - spacing - vehicle_half_length
        total_ds += spacing + vehicle_half_length*2.0

        # out range of the route or exceed maximum sampling range
        if vehicle_center_route_s < 0.0  or vehicle_center_route_s >= route_len_list[-1] or total_ds > max_range:
            break

        # find edge where current vehicle locate
        wp = route_utils.calc_waypoint_from_route_s(map_object, route, vehicle_center_route_s, route_len_list)

        # record new vehicles
        spawned_vehicle_list.append({
            "type_id": vehicle_type_id,
            "waypoint": wp
        })

        # update
        current_route_s = vehicle_center_route_s - vehicle_half_length

    return spawned_vehicle_list

