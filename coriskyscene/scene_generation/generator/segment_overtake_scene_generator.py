#!/usr/bin/env python
# -*- coding: utf-8 -*-
'''
@File    :   overtake_scene_generator.py
@Date    :   2024-01-17
@Author  :   Rongsong Li <rongsong.li@qq.com>
@Version :   1.0
@Desc    :   Generate overtake scenes   
'''

import os
import random
import logging
from lxml import etree

from coriskyscene.opendriveparser import parser as odr_parser
from coriskyscene.scene_generation import traffic_scene as trasc
from coriskyscene.scene_generation.generator.base_scene_sequential_generator \
    import BaseSceneSequentialGenerator
from coriskyscene.scene_generation.samplers import my_sampler
from coriskyscene.scene_generation.utils import scene_utils, route_utils


# ======================================================
# - Scene generator
# ======================================================

class OvertakingSceneGenerator(BaseSceneSequentialGenerator):
    """"
    
    Scene Desc
    ----------
    In the two-lane road, one lane for one direction, if ego (A) want to
    overtake the leading vehicle (B), it must use the neighboring lane of opposite 
    direction, and may collide with incoming vehicle (C) head to head.
        ----------------------------------------
                    A'--->       <--- C
        ----------------------------------------
         A --->   B ----->
        ----------------------------------------
    
    Atrributes
    ----------
    name : str
        The unique name of corresponding logical scenario. 

    """
    def __init__(self, name:str, map_path=None) -> None:
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
        """
        The ego is fixed at one position.
        """
        road_id,lane_id,s_pos = 4,-1,110
        wp_list = [self.map_object.getRoad(road_id).calcWaypoint(0, lane_id, s_pos)]

        return wp_list
    
    def get_occluder_waypoint_list(self, ego, **kwargs):
        """
        The occluder is in front of ego.
        """
        longitudinal_resolution = self.config.longitudinal_resolution
        eval_range = self.config.eval_range
        minimal_dist_to_leading_vehicle = 10  # meters, front bumper to rear bumper

        route = [[4, -1, 'start']]
        ego_front_route_s = route_utils.calc_route_s_for_actor(self.map_object, route, ego, 'front')
        init_s = ego_front_route_s + minimal_dist_to_leading_vehicle
        wp_list = route_utils.sample_waypoints_along_route(self.map_object, route, init_s, longitudinal_resolution)

        # filter by range
        wp_list = [wp for wp in wp_list if trasc.calc_distance_between_waypoints(wp, ego.waypoint)<=eval_range]

        return wp_list
    
    def get_occludee_waypoint_list(self, ego, occluder, **kwargs):
        """
        The occludee is in adjacent lane of reverse direction.
        """
        longitudinal_resolution = self.config.longitudinal_resolution
        eval_range = self.config.eval_range

        route = [[4, 1, 'end']]
        wp_list = route_utils.sample_waypoints_along_route(self.map_object, route, 0.0, longitudinal_resolution)

        # filter by range
        wp_list = [wp for wp in wp_list if trasc.calc_distance_between_waypoints(wp, ego.waypoint)<=eval_range]

        return wp_list


    def sample_others_ar(self, ego, occluder, occludee, seed, eval_range):
        """Auto regressively sample others
        TODO: get route automatically
        """
        self.oim.reset_to(3)  # there have been ego, occluder and occludee
        random.seed(seed)
        self.vehicle_sampler.set_seed(seed)

        all_other_vehicle_list = []

        # 1) sample following vehicles for ego
        route = [[4, -1, "start"]]
        route_s = route_utils.calc_route_s_for_actor(self.map_object, route, ego, 'rear')  # rear bumper
        # structure: [{"type_id": "vehicle.audi.a2", "waypoint": wp}]
        vehicle_list = scene_utils.sample_vehicles_against_route(self.map_object, route_s, route, self.vehicle_sampler, eval_range)
        all_other_vehicle_list.extend(vehicle_list)

        # 2) sample leading vehicles for occluder
        route = [[4, -1, "start"]]
        route_s = route_utils.calc_route_s_for_actor(self.map_object, route, occluder, 'front')  # front bumper
        vehicle_list = scene_utils.sample_vehicles_along_route(self.map_object, route_s, route, self.vehicle_sampler, eval_range)
        all_other_vehicle_list.extend(vehicle_list)

        # 3) sample leading/following vehicles for occludee
        route = [[4, 1, 'end']]
        rear_route_s = route_utils.calc_route_s_for_actor(self.map_object, route, occludee, 'rear')
        # following
        vehicle_list = scene_utils.sample_vehicles_against_route(self.map_object, rear_route_s, route, self.vehicle_sampler, eval_range)
        all_other_vehicle_list.extend(vehicle_list)
        # leading 
        d_lookback = 15  # meters, minimal distance exceeding which ego won't change lane
        front_route_s = route_utils.calc_route_s_for_actor(self.map_object, route, occludee, 'front')
        init_s = front_route_s + d_lookback + abs(ego.waypoint.s - occludee.waypoint.s)
        vehicle_list = scene_utils.sample_vehicles_along_route(self.map_object, init_s, route, self.vehicle_sampler, eval_range)
        all_other_vehicle_list.extend(vehicle_list)

        # Finally, convert to CarlaActor
        all_other_vehicle_list = [trasc.get_actor_from_actor_type(self.atm, ele['type_id'], self.oim, ele['waypoint']) 
                                  for ele in all_other_vehicle_list]

        return all_other_vehicle_list
    

if __name__ == '__main__':
    logging.basicConfig(format='%(levelname)s: %(message)s', level=logging.INFO)

    map_path = "/home/lrs/scripts/CoRiskyScene/coriskyscene/data_collection/data/map_opendrive/Town01.xodr"

    ot_gen = OvertakingSceneGenerator("overtake", map_path)
    ot_gen.run()