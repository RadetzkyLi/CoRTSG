#!/usr/bin/env python
# -*- coding: utf-8 -*-
'''
@File    :   onramp_scene_generator.py
@Date    :   2024-01-28
@Author  :   Rongsong Li <rongsong.li@qq.com>
@Version :   1.0
@Desc    :   Generate traffic scenes around on-ramp
'''

import os
import logging
import random
from lxml import etree

from coriskyscene.opendriveparser import parser as odr_parser
from coriskyscene.scene_generation import traffic_scene as trasc
from coriskyscene.scene_generation.generator.base_scene_sequential_generator \
    import BaseSceneSequentialGenerator
from coriskyscene.scene_generation.samplers import my_sampler
from coriskyscene.scene_generation.utils import scene_utils, route_utils


class OnrampSceneGenerator(BaseSceneSequentialGenerator):
    """
    
    Scene Desc
    ----------
    In highway's on-ramp, ego (A) is moving forward, occluded by trees (B),
    ego collides with another vehicle (C) that is moving forward to merge 
    into the main road.
    Vehicles on main road is supposed to be faster the vehicles on branch.
                    /    / 
                   / ↙C /
                  /    / 🌳🌳B
        ---------+    +--------------
                        <---- A
        -----------------------------
           <----
        -----------------------------
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

        # conflict point
        self.conflict_point = self.get_conflict_point()
        self.max_distance_to_conflict_point = self.config.eval_range

    
    def get_conflict_point(self):
        """
        Given a risky scenario in crossing or merging situcation, 
        the ego would have conflict with foe in the future at a 
        conflict point (or conflict area). 
        For more about conflict point, one can refer to the calculatation of 
        traffic safety metrics, e.g., TTC, PET.

        Refs
        ----
        1. https://sumo.dlr.de/docs/Simulation/Output/SSM_Device.html
        """
        # the first intersection point between lane center lines of the main road and the road branch
        road_id,lane_id,s_pos = 735,-6,30.0
        conflict_point = self.map_object.getRoad(road_id).calcWaypoint(0, lane_id, s_pos)

        return conflict_point
        
    def get_type_sampler_for(self, _type):
        """"Get actor type sampler"""
        if _type == "ego":
            return my_sampler.EgoTypeSampler()
        elif _type == "occluder":
            return my_sampler.StaticOccluderTypeSampler(['virtual.static.0002'])
        elif _type == "occludee":
            return my_sampler.OccludeeTypeSampler(target_main_classes=['vehicle'])
        else:
            raise ValueError("Unexpected `_type`(='{0}')".format(_type))

    def get_ego_waypoint_list(self, **kwargs):
        """"
        The ego is moving forward
        """
        longitudinal_resolution = self.config.longitudinal_resolution

        route = [[16, -6, 'start'], [25, -6, 'start']]
        wp_list = route_utils.sample_waypoints_along_route(self.map_object, route, 0.0, longitudinal_resolution)

        # ego shound't be far away from conflict point
        wp_list = [wp for wp in wp_list 
                   if trasc.calc_distance_between_waypoints(self.conflict_point, wp)<self.max_distance_to_conflict_point]

        return wp_list
    
    def get_occluder_waypoint_list(self, ego, **kwargs):
        """
        The occluder is virtual static.
        """
        occluder_half_length = 35  # meters
        length1 = 47.85  # length of road 16
        length2 = 18.80  # length of road 25
        s_pos = length1 - (occluder_half_length-length2)
        road_id,lane_id = 16, -8

        wp_list = [self.map_object.getRoad(road_id).calcWaypoint(0, lane_id, s_pos)]

        return wp_list
    
    def get_occludee_waypoint_list(self, ego, occluder, **kwargs):
        """
        The occludee is trying to merge into main road.
        """
        longitudinal_resolution = self.config.longitudinal_resolution
        eval_range = self.config.eval_range

        route = [[2, 3, 'end'], [45, 3, 'end'], [741, 3, 'end']]
        wp_list = route_utils.sample_waypoints_along_route(self.map_object, route, 0.0, longitudinal_resolution)

        # occludee shouldn't be far away from conflict point or ego
        wp_list = [wp for wp in wp_list if trasc.calc_distance_between_waypoints(wp, ego.waypoint)<eval_range]
        wp_list = [wp for wp in wp_list 
                   if trasc.calc_distance_between_waypoints(wp, self.conflict_point)<self.max_distance_to_conflict_point]

        return wp_list
    
    def sample_others_ar(self, ego, occluder, occludee, seed, eval_range):
        """Sample other road users in auto regressive manner.
        TODO get route automatically.
        
        """
        self.oim.reset_to(3)  # there have been ego, occluder and occludee
        random.seed(seed)
        self.vehicle_sampler.set_seed(seed)

        all_other_vehicle_list = []

        # 1) Sample ego's leading/following vehicles
        route = [[16, -6, 'start'], [25, -6, 'start'], [735, -6, 'start'], [28, -6, 'start']]
        
        # 1.1) lv: have passed the merging area
        minimal_dist_to_leading_vehicle = 15  # ego's front bumper to lv's center 
        ego_front_route_s = route_utils.calc_route_s_for_actor(self.map_object, route, ego, 'front')
        route_len_list = route_utils.get_route_len_list(self.map_object, route)
        init_s = max(ego_front_route_s + minimal_dist_to_leading_vehicle, route_len_list[3])
        vehicle_list = scene_utils.sample_vehicles_along_route(self.map_object, init_s, route, self.vehicle_sampler)
        all_other_vehicle_list.extend(vehicle_list)

        # 1.2) fv
        ego_rear_route_s = route_utils.calc_route_s_for_actor(self.map_object, route, ego, 'rear')
        init_s = ego_rear_route_s
        if init_s>0:
            vehicle_list = scene_utils.sample_vehicles_against_route(self.map_object, init_s, route, self.vehicle_sampler)
            all_other_vehicle_list.extend(vehicle_list)

        # 2) Sample occludee's following vehicles
        route = [[8, -3, 'start'], [2, 3, 'end'], [45, 3, 'end'], [741, 3, 'end']]
        occludee_rear_route_s = route_utils.calc_route_s_for_actor(self.map_object, route, occludee, 'rear')
        init_s = occludee_rear_route_s
        if init_s>0:
            vehicle_list = scene_utils.sample_vehicles_against_route(self.map_object, init_s, route, self.vehicle_sampler)
            all_other_vehicle_list.extend(vehicle_list)

        # 3) Sample vehicles on ego's neighboring lanes
        route_list = [
            [[16, -5, 'start'], [25, -5, 'start'], [735, -5, 'start'], [28, -5, 'start']],
            [[16, -4, 'start'], [25, -4, 'start'], [735, -4, 'start'], [28, -4, 'start']],
            [[16, -3, 'start'], [25, -3, 'start'], [735, -3, 'start'], [28, -3, 'start']]
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

        # 5) convert to CarlaActor
        all_other_vehicle_list = [
            self.get_actor_from_actor_type(self.atm, actor["type_id"], self.oim, actor["waypoint"]) 
            for actor in all_other_vehicle_list]
        
        return all_other_vehicle_list



if __name__ == '__main__':
    # generate scenes
    logging.basicConfig(format='%(levelname)s: %(message)s', level=logging.INFO)

    # replace as your local path
    map_path = "/home/lrs/scripts/CoRiskyScene/coriskyscene/data_collection/data/map_opendrive/Town06.xodr"

    gen = OnrampSceneGenerator("onramp", map_path)
    gen.run()