#!/usr/bin/env python
# -*- coding: utf-8 -*-
'''
@File    :   scene_vis.py
@Date    :   2024-01-10
@Author  :   Rongsong Li <rongsong.li@qq.com>
@Version :   1.0
@Desc    :   Visualize traffic scene
'''

import os
from lxml import etree
import numpy as np
from matplotlib import pyplot as plt 
import matplotlib.patches as mpatches

from coriskyscene.opendriveparser import parser as odr_parser
from coriskyscene.occlusion import occlusion_model as occm
from coriskyscene.scene_generation import traffic_scene as trasc


# ==========================================================
# - utility 
# ==========================================================
def isinstance_namedtuple(obj) -> bool:
    return (
            isinstance(obj, tuple) and
            hasattr(obj, '_asdict') and
            hasattr(obj, '_fields')
    )


# ==========================================================
# - visualization settings
# ==========================================================
LANETYPE_TO_COLOR = {
    "centerLine": 'black',
    "laneBorder": 'black',
    "crosswalk": 'gray',
    "driving": 'red',
    "sidewalk": 'red',
    "biking": 'red',
    "bidirectional": 'red',
    "parking": 'black',
    "default": "black"
}

LANETYPE_TO_HEADWIDTH = {
    "centerLine": 0.0,
    "laneBorder": 0.0,
    "crosswalk": 0.05,
    "driving": 0.05,
    "sidewalk": 0.0,
    "biking": 0.05,
    "bidirectional": 0.05,
    "parking": 0.0,
    "default": 0.0
}

OBJECT_ROLE_TO_COLOR = {
    "ego": "green",
    "occluder": "purple",
    "occludee": "red",
    "other": "black",
    "static": "gray"  # static obstacle
}


# ==========================================================================
# - Drawing
# ==========================================================================
def draw_bounding_box(ax, bbox, color="black", fill_box=False, alpha=None):
    """Draw bounding box"""
    if fill_box:
        # rectangle
        xy = bbox.vertices[2]  # left down corner
        rect = mpatches.Rectangle(
            xy, bbox.extent.x*2, bbox.extent.y*2, 
            rotation_point=[bbox.location.x, bbox.location.y],
            angle=bbox.rotation.yaw,
            alpha=alpha,
            facecolor=color,
            edgecolor=color
        )
        ax.add_patch(rect)
    else:
        for i in range(-1,3):
            ax.plot([bbox.vertices[i][0], bbox.vertices[i+1][0]], 
                    [bbox.vertices[i][1], bbox.vertices[i+1][1]], 
                    color=color)
        # draw arrow
        x_front = (bbox.vertices[0][0] + bbox.vertices[1][0])/2.0
        y_front = (bbox.vertices[0][1] + bbox.vertices[1][1])/2.0
        ax.plot([bbox.location.x, x_front],
                [bbox.location.y, y_front],
                color=color)
    
def draw_line_segment(ax, po, pa, r=120.0, color='blue'):
    """Draw line segment that starts from po and via pa. The 
    segment length is r.
    """
    vec_op = occm.get_vector_2d(po, pa)
    factor = r/occm.calc_length_2d(vec_op)
    vec_op = [factor*el for el in vec_op]
    ax.plot([po[0], po[0]+vec_op[0]], [po[1], po[1]+vec_op[1]], color=color)

def draw_waypoints(ax, wp_list, arrow_length: float=0.1):
    """Visualize waypoint"""
    for wp in wp_list:
        color = LANETYPE_TO_COLOR.get(wp.lane_type, LANETYPE_TO_COLOR["default"])
        head_width = LANETYPE_TO_HEADWIDTH.get(wp.lane_type, LANETYPE_TO_HEADWIDTH["default"])
        # ax.scatter(wp.coord[0], wp.coord[1], color=color, s=2)
        dx = arrow_length * np.cos(wp.heading)
        dy = arrow_length * np.sin(wp.heading)
        ax.arrow(wp.coord[0], wp.coord[1], dx, dy, head_width=head_width, fc=color, ec=color)

def draw_scene(scene, figsize=None, map_path=None, map_dir=None, 
               s_resolution:float=2.0, drawing_road_ids:list=None,
               title:str=None):
    """Visualize a traffic scene

    Parameters
    ----------
    scene : TrafficScene
        The scene to be visualized.

    figsize : list|tuple
        The size of the figure, e.g., (6.4, 4.8)

    map_path : str
        The OpenDRIVE map's path.

    map_dir : str
        The OpenDRIVE maps' directary. If `map_path` is None, we will search in the dir 
        of name provided by `scene`.

    drawing_road_ids : list
        Ids of roads to be drawed.
    
    """
    if isinstance_namedtuple(scene):
        scene = scene._asdict()

    # get the corresponding OpenDRIVE map
    if map_dir is None:
        map_dir = "/home/lrs/scripts/V2X-TSG/data_collection/data/map_opendrive" 
    if map_path is None:
        map_path = os.path.join(map_dir, scene["town_name"]+".xodr")
    with open(map_path, 'r') as f:
        odr_map = odr_parser.parse_opendrive(etree.parse(f).getroot())

    # retrive relevant roads
    if drawing_road_ids is None:
        drawing_road_ids = scene["relevant_roads"]
    lane_border_waypoints = []
    for road_id in set(drawing_road_ids):
        road = odr_map.getRoad(road_id)
        if road is None:
            raise ValueError("Map {0} has no road {1}!".format(scene["town_name"], road_id))
        lane_border_waypoints += road.getLaneBorder(s_resolution=s_resolution)
    
    # set font
    plt.rc('font', family='Times New Roman')
    fig,ax = plt.subplots(figsize=figsize) 
    ax.set_aspect('equal')
    
    # draw roads
    draw_waypoints(ax, lane_border_waypoints, arrow_length=s_resolution*0.6)
    
    # draw bbox
    ego_id = scene["ego_id"]
    occluder_ids = [pair[0] for pair in scene["occlusion_pairs"]]
    occludee_ids = [pair[1] for pair in scene["occlusion_pairs"]]
    for actor in scene["actor_list"]:
        # assign color for various vehicle
        if actor.id == ego_id:
            color = OBJECT_ROLE_TO_COLOR["ego"]
        elif actor.id in occluder_ids:
            color = OBJECT_ROLE_TO_COLOR["occluder"]
        elif actor.id in occludee_ids:
            color = OBJECT_ROLE_TO_COLOR["occludee"]
        else:
            color = OBJECT_ROLE_TO_COLOR["other"]
        draw_bounding_box(ax, actor.bounding_box, color=color)

    if title is None:
        title = "{0}: town={1}, roads={2}".format(scene["id"], scene["town_name"], sorted(scene["relevant_roads"]))

    plt.title(title)
    plt.xlabel('x [m]')
    plt.ylabel('y [m]')
    plt.show()
    plt.close()

    return lane_border_waypoints