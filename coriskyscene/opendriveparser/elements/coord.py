# -*- coding: utf-8 -*-
'''
@File    :   coord.py
@Time    :   2024-01-01
@Author  :   Rongsong Li <rongsong.li@qq.com>
@Version :   1.0
@Desc    :   common used elements in coordinate system
'''

from collections import namedtuple


Waypoint = namedtuple("Waypoint", 
                      ['coord', 'heading', 'lane_type', 
                       'road_id', 'section_id', 'lane_id', 
                       's', 'lane_width', 'lane_change'],
                       defaults=[None])
Waypoint.__doc__ = """
Args:
    coord: {ndarray}, [x, y, z] in meters
    heading: {float}, in radians
    lane_type: {str}, OpenDRIVE Lanetype + ["laneBorder", "centerLine", "crosswalk"]
    road_id: {int}, the road id in OpenDRIVE file
    section_id: {int}, OpenDRIVE section's id, based on the order that they are originally defined.
    lane_id: {int}, OpenDRIVE lane's id, this value can be positive or negative which represents 
        the direction of the current lane with respect to the road
    s: {float}, OpenDRIVE s value of the current position
    lane_width: {float}, horizental size of the road at current s.
    lane_change: {'left', 'right', 'both', None}, the permission to turn,
"""
