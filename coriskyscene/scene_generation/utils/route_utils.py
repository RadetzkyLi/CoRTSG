#!/usr/bin/env python
# -*- coding: utf-8 -*-
'''
@File    :   route_utils.py
@Date    :   2024-01-23
@Author  :   Rongsong Li <rongsong.li@qq.com>
@Version :   1.0
@Desc    :   Utility for route
'''

import os
import sys
import numpy as np



# ========================================================================
# - Conversion between route_s and waypoint
# ========================================================================
def calc_route_s_from_waypoint(map_object, route, waypoint):
    """Calculate the s value for given waypoint. The s denotes the distance
    from route's start to current location along the route's s-axis.
    
    Returns
    -------
    s : float
        Returns None if waypoint out range of the route
    """
    s = None
    prev_s = 0
    for edge in route:
        # find lane where the actor locate
        if edge[0] == waypoint.road_id and edge[1] == waypoint.lane_id:
            if edge[2] == 'start':
                ds_on_lane = waypoint.s 
            elif edge[2] == 'end':
                ds_on_lane = map_object.getRoad(edge[0]).length - waypoint.s
            else:
                raise ValueError("Unexpected contact point '{0}', expected one of 'start', 'end'".format(edge[2]))
            s = ds_on_lane + prev_s
            break

        prev_s += map_object.getRoad(edge[0]).length
    return s

def calc_route_s_for_actor(map_object, route, actor, loc="center"):
    """Calculate the s position of actor along given route. By default,
    we return the location of actor's center.
    The actor's heading is assumed to be consistent with the route.

    Parameters
    ----------
    map_object : 
        OpenDRIVE's map

    route : list
        A list of (road_id, lane_id, contactPoint). The route's direction is consistent
        with lane's driving direction.

    actor : CarlaActor

    loc : str
        {"center", "front", "rear"}
    
    Returns
    -------
    s : float
        Returns None if actor out of the route

    """
    s = calc_route_s_from_waypoint(map_object, route, actor.waypoint)
    if s is None:
        return s
    
    # offset to front or rear
    if loc == 'center':
        pass
    elif loc == 'front':
        s += actor.bounding_box.extent.x 
    elif loc == 'rear':
        s -= actor.bounding_box.extent.x 
    else:
        raise NotImplemented("Unsupported loc={0}".format(loc))
    
    return s

def calc_waypoint_from_route_s(map_object, route, s, route_len_list=None):
    """
    
    waypoint : Waypoint
        The waypoint at that position. Returns None if `s` out range of the route.
    """
    if route_len_list is None:
        route_len_list = get_route_len_list(map_object, route)
    # 1) find edge where the `s` locate
    current_edge_index = find_target_edge_in_route(map_object, route, s, route_len_list)
    if current_edge_index is None:
        return None

    # 2) calculate s_pos for target road
    road = map_object.getRoad(route[current_edge_index][0])
    lane_id = route[current_edge_index][1]
    
    # distance occupied on the edge
    ds_edge = s - route_len_list[current_edge_index]
    contact_point = route[current_edge_index][2]
    
    if contact_point == 'start':
        s_pos = ds_edge
    elif contact_point == 'end':
        s_pos = road.length - ds_edge
    else:
        raise ValueError("contact point can only be 'start' or 'end', got {0}".format(contact_point))
    
    # 3) calculate waypoint
    waypoint = road.calcWaypoint(0, lane_id, s_pos)
    return waypoint


def get_route_len_list(map_object, route):
    """Calculat the route's accumulated length list.
    
    Parameters
    ----------
    map_object : 
        The OpenDRIVE's map

    route : list
        A list of (road_id, lane_id, contactPoint).

    Returns
    -------
    route_len_list : list[float]
        e.g., [0, l1, l1+l2, l1+l2+l3, ...]
    """
    # accumulated_route_length = 0.0
    # for edge in route:
    #     road = map_object.getRoad(edge[0])
    #     accumulated_route_length += road.length 
    #     if s <= accumulated_route_length:
    #         return edge
    
    route_len_list = [0]
    for edge in route:
        road = map_object.getRoad(edge[0])
        route_len_list.append(road.length + route_len_list[-1])

    return route_len_list

def find_target_edge_in_route(map_object, route, s, route_len_list=None):
    """Find the target edge in the route.
    TODO: check direction's consistency of edges of the route.
    
    Parameters
    ----------
    map_object : 
        The OpenDRIVE map

    route : list
        A lane as a edge and successive edges composed of a route. It's supposed 
        that their driving direction is consistent. 
        route: edge1 --> edge2 --> edge2 --> ...

    s : float
        Distance from route's start point to current location

    route_len_list : list
        Accumulated length list of route. Supposed a route with two edges of length 
        l1 and l2, then route_len_list = [0, l1, l1+l2]

    Returns
    -------
    current_edge_idnex : int
        Returns None if `s` exceed the route length

    """
    assert s>=0.0

    if route_len_list is None:
       route_len_list = get_route_len_list(map_object, route)

    if s>= route_len_list[-1]:
        return None
    
    # 1) find edge where the `s` locate
    route_len_list = np.array(route_len_list)
    mask = route_len_list > s 
    sub_idx = np.argmin(route_len_list[mask] - s)
    current_edge_index = np.arange(len(route_len_list))[mask][sub_idx] - 1

    return current_edge_index


# ==========================================================================
# - sample wayponts
# ==========================================================================
def sample_waypoints_along_route(map_object, route, init_s:float, longitudinal_resolution:float, lateral_resolution:float=None):
    """Sample waypoints along given route according to specified params.
    
    Parameters
    ----------
    map_object :
        The OpenDRIVE map

    route : 

    init_s : 
        The s value from route's start to current location.

    lonitudinal_resolution : float
        along s-axis, in meters

    lateral_resolution : float
        along t-axis, in meters

    Returns
    -------
    wp_list : list

    """
    assert init_s>=0.0
    assert longitudinal_resolution>0.02
    assert lateral_resolution is None or lateral_resolution>0.02

    if lateral_resolution is None:
        route_len_list = get_route_len_list(map_object, route)
        route_length = route_len_list[-1]
        wp_list = [calc_waypoint_from_route_s(map_object, route, s, route_len_list)
                   for s in np.arange(init_s, route_length, longitudinal_resolution)]
    else:
        # TODO implement for sublane mode
        wp_list = []

    return wp_list

def sample_waypoints_against_route(map_object, route, init_s:float, longitudinal_resolution:float, lateral_resolution:float=None):
    """Sample waypoints against the given route.
    
    Parameters
    ----------
    map_object : 

    route :

    init_s :
        The s value from route's start to current location.

    longitudinal_resolution : float
        along s-axis, in meters

    lateral_resolution : float
        along t-axis, in meters

    Returns
    -------
    wp_list : list
        A list of Waypoints
    """
    assert init_s>0.0
    assert longitudinal_resolution>0.02
    assert lateral_resolution is None or lateral_resolution>0.02

    if lateral_resolution is None:
        route_len_list = get_route_len_list(map_object, route)
        wp_list = [calc_waypoint_from_route_s(map_object, route, s, route_len_list)
                   for s in np.arange(init_s, 0.0, -longitudinal_resolution)]
    else:
        # TODO implement for sublane mode
        wp_list = []

    return wp_list