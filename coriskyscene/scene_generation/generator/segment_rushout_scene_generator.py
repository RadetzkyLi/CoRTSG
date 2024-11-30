#!/usr/bin/env python
# -*- coding: utf-8 -*-
'''
@File    :   segment_rushout_scene_generator.py
@Date    :   2024-01-20
@Author  :   Rongsong Li <rongsong.li@qq.com>
@Version :   1.0
@Desc    :   Generate rushout scene on segment or junction
'''

import os
import logging
import random
from lxml import etree
import numpy as np

from coriskyscene.opendriveparser import parser as odr_parser
from coriskyscene.scene_generation import traffic_scene as trasc
from coriskyscene.scene_generation.generator.base_scene_sequential_generator \
    import BaseSceneSequentialGenerator
from coriskyscene.scene_generation.samplers import my_sampler
from coriskyscene.scene_generation.utils import scene_utils, route_utils

class SegmentRushoutSceneGenerator(BaseSceneSequentialGenerator):
    """Rush out scene geneartion on road segment.
    
    Scene Desc
    ----------
    In road with more than two lanes of same direction, occluded by one 
    parking vehicle (B), the moving forward ego (A) collides with a walker
    who is rushing into road from front of B.

    ---------------------------------------------------
             A ---->             (lv)---->
    ---------------------------------------------------
                    B ====>  ↑ (C)
    ---------------------------------------------------

    
    """
    def __init__(self, name: str, map_path=None) -> None:
        # load opendrive map
        with open(map_path, 'r') as f:
            map_object = odr_parser.parse_opendrive(etree.parse(f).getroot())
        super().__init__(name, map_object)

        self.map_name = os.path.basename(map_path).split(".xodr")[0]

        data_dir = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "data")
        # spacing distribution
        spacing_dist_path = os.path.join(data_dir, "spacing_distri_SPMD1.pkl")
        self.vehicle_sampler = my_sampler.VehicleSampler(
            self.atm,
            spacing_dist_path
        )

        # set save dir
        self.set_save_dir(data_dir)

        # update config
        self.config.occluder_as_cav = False

    def get_type_sampler_for(self, _type):
        """"Get actor type sampler"""
        if _type == "ego":
            return my_sampler.EgoTypeSampler()
        elif _type == "occluder":
            return my_sampler.OccluderTypeSampler()
        elif _type == "occludee":
            return my_sampler.OccludeeTypeSampler(target_main_classes=['walker'])
        else:
            raise ValueError("Unexpected `_type`(='{0}')".format(_type))

    def get_ego_waypoint_list(self):
        """The ego is fixed on a position"""
        road_id = 3
        lane_id = -1
        ego_s = 30

        road = self.map_object.getRoad(road_id)
        wp_list = [road.calcWaypoint(0, lane_id, ego_s)]

        return wp_list

    def get_occluder_waypoint_list(self, ego):
        """Calculate waypoints for occluder, which is on adjacent right lane of ego.
        """
        longitudinal_resolution = self.config.longitudinal_resolution
        eval_range = self.config.eval_range

        route = [[3, -2 , 'start']]
        route_parallel = [[3, -1, 'start']]  # ego's route
        ego_front_route_s = route_utils.calc_route_s_for_actor(self.map_object, route_parallel, ego, 'front')
    
        wp_list = route_utils.sample_waypoints_along_route(self.map_object, route, ego_front_route_s, longitudinal_resolution)
        
        # filter out waypoints out range of ego's eval range
        wp_list = [wp_tmp for wp_tmp in wp_list if trasc.calc_distance_between_waypoints(wp_tmp, ego.waypoint)<=eval_range]

        return wp_list

    def get_occludee_waypoint_list(self, ego, occluder):
        """"
        Calculate waypoints for occludee, which is in front of occluder.
        
        """
        lateral_resulution =  self.config.lateral_resolution

        # get occluder's info
        road_id = occluder.waypoint.road_id
        lane_id = occluder.waypoint.lane_id
        road = self.map_object.getRoad(road_id)
        occluder_half_length = occluder.bounding_box.extent.x
        pedestrian_half_width = 0.25 # meters

        dist_to_pedestrian = 1.0  # meters
        dist_to_center = dist_to_pedestrian + occluder_half_length + pedestrian_half_width

        if lane_id < 0:
            s_pos = occluder.waypoint.s + dist_to_center
        else:
            s_pos = occluder.waypoint.s - dist_to_center

        wp_list = road.calcWaypointInSublaneCenter(s_pos, lateral_resulution, target_lane_types=['driving'])
        wp_list = [wp for wp in wp_list if wp.lane_id == lane_id]

        # rotate pedestrian 90 degrees anti-clockwise complying with that 
        # the pedestrian is rushing out from right to left
        wp_list = [wp._replace(heading=wp.heading+0.5*np.pi) for wp in wp_list]

        return wp_list
    
    def sample_others_ar(self, ego, occluder, occludee, seed, eval_range):
        """Auto regressively sample others
        TODO: get route automatically
        """
        self.oim.reset_to(3)
        random.seed(seed)
        self.vehicle_sampler.set_seed(seed)

        all_other_vehicle_list = []

        # 1) Sample ego's leading/following vehicles
        # 1.1) lv: must be ahead of occludee by some marginal
        route = [[3,-1, "start"], [875, -1, "start"]]
        # occludee's route, same direction as ego's route
        route_parallel = [[3, -2, 'start'], [875, -2, 'start']] 
        init_s = route_utils.calc_route_s_for_actor(self.map_object, route, ego, 'front')
        dist_marginal = 10  # meters
        
        # structure: [{"type_id": "vehicle.audi.a2", "waypoint": wp}]
        vehicle_list = scene_utils.sample_vehicles_along_route(self.map_object, init_s, route, self.vehicle_sampler, eval_range)
        occludee_s_in_route = route_utils.calc_route_s_for_actor(self.map_object, route_parallel, occludee)
        vehicle_list = [veh for veh in vehicle_list 
                        if route_utils.calc_route_s_from_waypoint(self.map_object, route, veh['waypoint']) > occludee_s_in_route+dist_marginal]
        all_other_vehicle_list.extend([
            trasc.get_actor_from_actor_type(self.atm, ele['type_id'], self.oim, ele['waypoint'])
            for ele in vehicle_list
        ])

        # 1.2) fv
        init_s = route_utils.calc_route_s_for_actor(self.map_object, route, ego, 'rear')  # rear bumper
        vehicle_list = scene_utils.sample_vehicles_against_route(self.map_object, init_s, route, self.vehicle_sampler, eval_range)
        all_other_vehicle_list.extend([
            trasc.get_actor_from_actor_type(self.atm, ele['type_id'], self.oim, ele['waypoint'])
            for ele in vehicle_list
        ])

        # 2) Sample vehicles on neighboring routes
        route_list = [
            [[874, 1, 'end'], [3, 1, 'end']],
            [[874, 2, 'end'], [3, 2, 'end']],
            [[874, 3, 'end'], [3, 3, 'end']]
        ]
        for route in route_list:
            vehicle_list = scene_utils.sample_vehicles_along_route(self.map_object, 0, route, self.vehicle_sampler, max_range=200)
            vehicle_list = [trasc.get_actor_from_actor_type(self.atm, ele['type_id'], self.oim, ele['waypoint']) 
                            for ele in vehicle_list]
            vehicle_list = [veh for veh in vehicle_list if ego.distance(veh) < eval_range]
            all_other_vehicle_list.extend(vehicle_list)

        return all_other_vehicle_list
    
    
if __name__ == '__main__':
    # generate scenes
    logging.basicConfig(format='%(levelname)s: %(message)s', level=logging.INFO)

    # replace as your local path
    map_path = "/home/lrs/scripts/CoRiskyScene/coriskyscene/data_collection/data/map_opendrive/Town03.xodr"

    rushout_gen = SegmentRushoutSceneGenerator("segrush", map_path)
    rushout_gen.run()