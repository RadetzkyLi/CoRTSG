#!/usr/bin/env python
# -*- coding: utf-8 -*-
'''
@File    :   lc_scene_generator.py
@Date    :   2024-01-25
@Author  :   Rongsong Li <rongsong.li@qq.com>
@Version :   1.0
@Desc    :   Lane Change Scene Generator
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


class LaneChangeSceneGenerator(BaseSceneSequentialGenerator):
    """
    
    Scene Desc
    ----------
    In road straight segment with more than one lanes per direction, ego (A)
    is going to change to its adjacent left lane, occluded by its following 
    vehicle (B) on the same lane, ego collides with another following vehicle
    (C) on target lane.

        ----------------------------------------
                    <---- npc
        ++++++++++++++++++++++++++++++++++++++++
           C ---->                      A' ---->
        ----------------------------------------
                       B ---->   A ----> 
        ----------------------------------------
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
        

    def get_type_sampler_for(self, _type):
        """"Get actor type sampler"""
        if _type == "ego":
            return my_sampler.EgoTypeSampler()
        elif _type == "occluder":
            return my_sampler.OccluderTypeSampler()
        elif _type == "occludee":
            return my_sampler.OccludeeTypeSampler(target_main_classes=['vehicle'])
        else:
            raise ValueError("Unexpected `_type`(='{0}')".format(_type))

    def get_ego_waypoint_list(self, **kwargs):
        """"
        Fix ego at one position.
        """
        road_id = 1
        lane_id = -2
        ego_s = 10.0  # meters

        road = self.map_object.getRoad(road_id)
        wp_list = [road.calcWaypoint(0, lane_id, ego_s)]

        return wp_list
    
    def get_occluder_waypoint_list(self, ego, **kwargs):
        """"
        The occluder is the following vehicle of ego

        TODO: get route automatically
        """
        longitudinal_resolution = self.config.longitudinal_resolution
        eval_range = self.config.eval_range
        occluder_half_length = 4.5  # meters
        minimal_dist_to_leading_vehicle = 5 # meters, front bumper to rear bumper

        route = [[516, 2, 'end'], [4, 2, 'end'], [8, -2, 'start'], [1, -2, 'start']]
        ego_route_s = route_utils.calc_route_s_for_actor(self.map_object, route, ego, 'rear')
        init_s = ego_route_s - minimal_dist_to_leading_vehicle - occluder_half_length
        wp_list = route_utils.sample_waypoints_against_route(self.map_object, route, init_s, longitudinal_resolution)

        # filter points out of range
        wp_list = [wp for wp in wp_list if trasc.calc_distance_between_waypoints(wp, ego.waypoint)<eval_range]

        return wp_list
    
    def get_occludee_waypoint_list(self, ego, occluder, **kwargs):
        """
        The occludee is on the adjacent left lane of ego and behind ego.

        TODO: get route automatically
        """
        longitudinal_resolution = self.config.longitudinal_resolution
        eval_range = self.config.eval_range

        route = [[516, 1, 'end'], [4, 1, 'end'], [8, -1, 'start'], [1, -1, 'start']]
        route_parallel = [[516, 2, 'end'], [4, 2, 'end'], [8, -2, 'start'], [1, -2, 'start']]
        ego_route_s = route_utils.calc_route_s_for_actor(self.map_object, route_parallel, ego)
        
        wp_list = route_utils.sample_waypoints_against_route(self.map_object, route, ego_route_s, longitudinal_resolution)

        # filter points out of range
        wp_list = [wp for wp in wp_list if trasc.calc_distance_between_waypoints(wp, ego.waypoint)<eval_range]

        return wp_list
    
    def sample_others_ar(self, ego, occluder, occludee, seed, eval_range):
        """Sample other road users in auto regressive manner.
        TODO get route automatically.
        
        """
        self.oim.reset_to(3)   # there have been ego, occluder and occludee
        random.seed(seed)
        self.vehicle_sampler.set_seed(seed)

        all_other_vehicle_list = []

        # 1) Sample ego's leading vehicles
        minimal_dist_to_leading_vehicle = 15  # ego's front bumper to lv's center when change lane
        ego_route = [[516, 2, 'end'], [4, 2, 'end'], [8, -2, 'start'], [1, -2, 'start']]
        ego_front_route_s = route_utils.calc_route_s_for_actor(self.map_object, ego_route, ego, 'front')
        init_s = ego_front_route_s + minimal_dist_to_leading_vehicle
        vehicle_list = scene_utils.sample_vehicles_along_route(self.map_object, init_s, ego_route, self.vehicle_sampler)
        all_other_vehicle_list.extend(vehicle_list)

        # 2) Sample occluder's following vehicles
        occluder_rear_route_s = route_utils.calc_route_s_for_actor(self.map_object, ego_route, occluder, 'rear')
        vehicle_list = scene_utils.sample_vehicles_against_route(self.map_object, occluder_rear_route_s, ego_route, self.vehicle_sampler)
        all_other_vehicle_list.extend(vehicle_list)

        # 3) Sample occludee's leading and following vehicles
        # the route is parallel to that of ego
        route = [[516, 1, 'end'], [4, 1, 'end'], [8, -1, 'start'], [1, -1, 'start']]
        # 3.1) lv
        d_lookahead = 35  # the minimal distance (in meters) to adjacent leading vehicle exceeding which ego won't change lane
        vehicle_list = scene_utils.sample_vehicles_along_route(self.map_object, ego_front_route_s+d_lookahead, route, self.vehicle_sampler)
        all_other_vehicle_list.extend(vehicle_list)
        # 3.2) fv
        occludee_rear_route_s = route_utils.calc_route_s_for_actor(self.map_object, route, occludee, 'rear')
        vehicle_list = scene_utils.sample_vehicles_against_route(self.map_object, occludee_rear_route_s, route, self.vehicle_sampler)

        # 4) Sample other vehilcles of reverse direction
        route_list = [
            [[1, 1, 'end'], [8, 1, 'end'], [4, -1, 'start'], [515, -1, 'start']],
            [[1, 2, 'end'], [8, 2, 'end'], [4, -2, 'start'], [515, -2, 'start']]
        ]
        for route in route_list:
            vehicle_list = scene_utils.sample_vehicles_along_route(self.map_object, 0.0, route, self.vehicle_sampler, max_range=200.0)
            all_other_vehicle_list.extend(vehicle_list)

        # 5) Filter vehicles by range
        center_actor = {
            "type_id": ego.type_id,
            "waypoint": ego.waypoint
        }
        all_other_vehicle_list = self.filter_actors_by_range(all_other_vehicle_list, center_actor, eval_range)

        # 6) convert to CarlaActor
        all_other_vehicle_list = [
            self.get_actor_from_actor_type(self.atm, actor["type_id"], self.oim, actor["waypoint"]) 
            for actor in all_other_vehicle_list]
        
        return all_other_vehicle_list


if __name__ == '__main__':
    # generate scenes
    logging.basicConfig(format='%(levelname)s: %(message)s', level=logging.INFO)

    # replace as your local path
    map_path = "/home/lrs/scripts/CoRiskyScene/coriskyscene/data_collection/data/map_opendrive/Town10HD.xodr"

    gen = LaneChangeSceneGenerator("lanechange", map_path)
    gen.run()
