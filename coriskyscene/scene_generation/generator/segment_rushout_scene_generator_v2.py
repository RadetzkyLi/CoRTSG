#!/usr/bin/env python
# -*- coding: utf-8 -*-
'''
@File    :   segment_rushout_scene_generator_v2.py
@Date    :   2024-01-23
@Author  :   Rongsong Li <rongsong.li@qq.com>
@Version :   1.0
@Desc    :   Another scene generator for walker/biker crossing road segment
'''

import os
import time
import logging
import random
from lxml import etree

from coriskyscene.opendriveparser import parser as odr_parser
from coriskyscene.scene_generation import traffic_scene as trasc
from coriskyscene.scene_generation.generator.base_scene_sequential_generator \
    import BaseSceneSequentialGenerator
from coriskyscene.scene_generation.samplers import my_sampler
from coriskyscene.scene_generation.utils import scene_utils, route_utils


class SegmentRushoutSceneGeneratorV2(BaseSceneSequentialGenerator):
    """
    Scene Desc
    ----------
    In road segment with zebra marking, the ego (A) is going straight, 
    occluded by slow vehicle (B) on adjacent left lane, so the ego 
    collide with walker/biker (C) that are crossing.

        ---------------------------------------------------====-------------------
                        <----                              ====
        ---------------------------------------------------====-------------------
                <---- (npc)                                ====
        ---------------------------------------------------=↓== (C: walker/biker)
                                                B ---->    ====
        ---------------------------------------------------====-------------------
                                       A ---->             ====
        ---------------------------------------------------====-------------------


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
        self.config.occluder_as_cav = True

    def get_type_sampler_for(self, _type):
        """"Get actor type sampler"""
        if _type == "ego":
            return my_sampler.EgoTypeSampler()
        elif _type == "occluder":
            return my_sampler.OccluderTypeSampler()
        elif _type == "occludee":
            return my_sampler.OccludeeTypeSampler(target_main_classes=['nonmotor'])
        else:
            raise ValueError("Unexpected `_type`(='{0}')".format(_type))

    def get_ego_waypoint_list(self):
        """Ego is on adjacent right lane of occluder."""
        route = [[3, -2, "start"]]
        longitudinal_resolution = 2.0 # meters
        wp_list = route_utils.sample_waypoints_along_route(self.map_object, route, 0.0, longitudinal_resolution)

        return wp_list
    
    def get_occluder_waypoint_list(self, ego):
        """Fix the occluder on one position"""
        road_id,lane_id = 3,-1
        s = 80.0
        road = self.map_object.getRoad(road_id)
        wp_list = [road.calcWaypoint(0, lane_id, s)]

        return wp_list
    
    def get_occludee_waypoint_list(self, ego, occluder):
        """The occludeee is in front of the occluder."""
        dist_to_front_bumper = 1.0 # meters
        occludee_half_length = 0.95 # meters
        lateral_resolution = self.config.lateral_resolution
        eval_range = self.config.eval_range
        route = [[3, -1, "start"]]
        
        # find road where occludee locate
        occluder_front_route_s = route_utils.calc_route_s_for_actor(self.map_object, route, occluder, loc="front")
        occludee_center_route_s = occluder_front_route_s + dist_to_front_bumper + occludee_half_length
        wp = route_utils.calc_waypoint_from_route_s(self.map_object, route, occludee_center_route_s)  # lane center

        # calculate waypoints in sublane mode
        road = self.map_object.getRoad(wp.road_id)
        wp_list = road.calcWaypointInSublaneCenter(wp.s, lateral_resolution=lateral_resolution, target_lane_types=["driving"])
        wp_list = [wp_tmp for wp_tmp in wp_list if wp_tmp.lane_id == wp.lane_id]

        # filter out waypoints out range of ego's eval range
        wp_list = [wp_tmp for wp_tmp in wp_list if trasc.calc_distance_between_waypoints(wp_tmp, ego.waypoint)<=eval_range]

        return wp_list

    def sample_others_ar(self, ego, occluder, occludee, seed, eval_range):
        """Sample other road users in auto regressive manner.
        TODO get route automatically.
        
        """
        self.oim.reset_to(3)
        random.seed(seed)
        self.vehicle_sampler.set_seed(seed)

        all_other_vehicle_list = []

        # 1) Sample ego's following vehicles
        route = [[3, -2, "start"]]
        route_s = route_utils.calc_route_s_for_actor(self.map_object, route, ego, 'rear')
        # the ego's rear lie in route
        if route_s>=0:
            vehicle_list = scene_utils.sample_vehicles_against_route(self.map_object, route_s, route, self.vehicle_sampler, eval_range)
            all_other_vehicle_list.extend(vehicle_list)

        # 2) Sample occluder's leading/following vehicles
        route = [[3, -1, 'start']]
        # 2.1) fv
        route_s = route_utils.calc_route_s_for_actor(self.map_object, route, occluder, 'rear')
        vehicle_list = scene_utils.sample_vehicles_against_route(self.map_object, route_s, route, self.vehicle_sampler, eval_range)
        all_other_vehicle_list.extend(vehicle_list)
        # 2.2) lv
        route_s = route_utils.calc_route_s_for_actor(self.map_object, route, occluder, loc='front')
        crossing_width = 4.0  # meters
        vehicle_list = scene_utils.sample_vehicles_along_route(self.map_object, route_s+crossing_width, route, self.vehicle_sampler)
        all_other_vehicle_list.extend(vehicle_list)

        # 3) Sample vehicle on neighboring lanes of the same road but of reverse direction
        route_list = [
            [[874, 1, 'end'], [3, 1, 'end']],
            [[874, 2, 'end'], [3, 2, 'end']],
            [[874, 3, 'end'], [3, 3, 'end']]
        ]
        for route in route_list:
            vehicle_list = scene_utils.sample_vehicles_along_route(self.map_object, 0.0, route, self.vehicle_sampler)
            all_other_vehicle_list.extend(vehicle_list)

        # 4) Filter vehicles by range
        center_actor = {
            "type_id": ego.type_id,
            "waypoint": ego.waypoint
        }
        all_other_vehicle_list = self.filter_actors_by_range(all_other_vehicle_list, center_actor, eval_range)

        # convert to CarlaActor
        all_other_vehicle_list = [
            self.get_actor_from_actor_type(self.atm, actor["type_id"], self.oim, actor["waypoint"]) 
            for actor in all_other_vehicle_list]
        
        return all_other_vehicle_list
    

if __name__ == '__main__':
    # generate scenes
    logging.basicConfig(format='%(levelname)s: %(message)s', level=logging.INFO)

    # replace as your local path
    map_path = "/home/lrs/scripts/CoRiskyScene/coriskyscene/data_collection/data/map_opendrive/Town03.xodr"

    rushout_gen = SegmentRushoutSceneGeneratorV2("segrush_v2", map_path)
    rushout_gen.run()
