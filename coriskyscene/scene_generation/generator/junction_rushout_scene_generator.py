#!/usr/bin/env python
# -*- coding: utf-8 -*-
'''
@File    :   junction_rushout_scene_generator.py
@Date    :   2024-01-22
@Author  :   Rongsong Li <rongsong.li@qq.com>
@Version :   1.0
@Desc    :   Generating scene in which walker/biker rush out in junction area
'''


import os
import sys
import logging
import random
from lxml import etree
import numpy as np

from coriskyscene.opendriveparser import parser as odr_parser
from coriskyscene.occlusion import occlusion_model as occm
from coriskyscene.scene_generation import traffic_scene as trasc
from coriskyscene.scene_generation.generator.base_scene_sequential_generator \
    import BaseSceneSequentialGenerator
from coriskyscene.scene_generation.samplers import my_sampler
from coriskyscene.scene_generation.utils import scene_utils, route_utils


class JuncitonRushoutSceneGenerator(BaseSceneSequentialGenerator):
    """
    
    Scene Desc
    ----------
    In junction with crosswalk, the ego car is driving straight and passing
    the crosswalk, occluded by another vehicle waiting for turning left on ego's
    left lane, so the ego collide with the crossing walker/biker.

                                 |||||||||||||||||
        ---------------------====                 ====
                 <---        ====                 ====
        +++++++++++++++++++++====                 ====
          (waiting veh) ---> =↓==(walker)         ====
        ---------------------====                 ====
            (ego) --->       ====                 ====
        ---------------------====                 ====
                                 |||||||||||||||||
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
        # there is RSU, save scene anyway if occlusion appears
        self.config.ignore_none_valid_cav = True

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
        """The ego is on adjacent right lane of the occluder."""
        route = [[16, -2, "start"], [25, -2, "start"]]
        longitudinal_resolution = self.config.longitudinal_resolution
        wp_list = route_utils.sample_waypoints_along_route(self.map_object, route, 0.0, longitudinal_resolution)
        
        return wp_list

    def get_occluder_waypoint_list(self, ego):
        """The occluder is fixed on entry of the junction."""

        road_id = 25
        lane_id = -1
        occluder_half_length = 3.0 # meters

        road = self.map_object.getRoad(road_id)
        wp_list = [road.calcWaypoint(0, lane_id, road.length-occluder_half_length)]

        return wp_list

    def get_occludee_waypoint_list(self, ego, occluder):
        """The occludee is in front of the occluder."""
        dist_to_front_bumper = 1.5 # meters
        occludee_half_length = 0.95 # meters
        lateral_resolution = self.config.lateral_resolution
        eval_range = self.config.eval_range
        route = [[25, -1, "start"], [1675, -1, "start"]]
        
        # find road where occludee locate
        occluder_front_route_s = route_utils.calc_route_s_for_actor(self.map_object, route, occluder, loc="front")
        occludee_center_route_s = occluder_front_route_s + dist_to_front_bumper + occludee_half_length
        wp = route_utils.calc_waypoint_from_route_s(self.map_object, route, occludee_center_route_s)  # lane center

        # calculate waypoints in sublane mode
        road = self.map_object.getRoad(wp.road_id)
        wp_list = road.calcWaypointInSublaneCenter(wp.s, lateral_resolution=lateral_resolution, target_lane_types=["driving"])
        # wp_list = [wp_tmp for wp_tmp in wp_list if wp_tmp.lane_id == wp.lane_id]

        # filter out waypoints out range of ego's eval range
        wp_list = [wp_tmp for wp_tmp in wp_list if trasc.calc_distance_between_waypoints(wp_tmp, ego.waypoint)<=eval_range]

        # rotate walker/biker 90 degrees clockwise complying with that 
        # they are crossing
        wp_list = [wp._replace(heading=wp.heading-0.5*np.pi) for wp in wp_list]

        return wp_list
        
    def sample_ego(self, sampler):
        return self.sample_actor_sequentially(sampler, 0)
    
    def sample_occluder(self, sampler):
        return self.sample_actor_sequentially(sampler, 1)
    
    def sample_occludee(self, sampler):
        return self.sample_actor_sequentially(sampler, 2)
    
    def random_sample_vehicle_at(self, road_id, lane_id, s):
        type_id = self.vehicle_sampler.random_select_vehicle_type().id
        road = self.map_object.getRoad(road_id)
        wp = road.calcWaypoint(0, lane_id, s)
        return {"type_id": type_id, "waypoint": wp}

    def sample_others_ar(self, ego, occluder, occludee, seed, eval_range):
        """Sample other road users in auto regressive manner.
        TODO get route automatically.
        
        """
        self.oim.reset_to(3)
        random.seed(seed)
        self.vehicle_sampler.set_seed(seed)

        all_other_vehicle_list = []

        # 1) Sample ego's following vehicles
        route = [[16, -2, "start"], [25, -2, "start"]]
        route_s = route_utils.calc_route_s_for_actor(self.map_object, route, ego, 'rear')
        # the ego's rear lie in route
        if route_s>=0:
            vehicle_list = scene_utils.sample_vehicles_against_route(self.map_object, route_s, route, self.vehicle_sampler, eval_range)
            all_other_vehicle_list.extend(vehicle_list)

        # 2) Sample occluder's following vehicles
        route = [[16, -1, 'start'], [25, -1, 'start']]
        route_s = route_utils.calc_route_s_for_actor(self.map_object, route, occluder, 'rear')
        vehicle_list = scene_utils.sample_vehicles_against_route(self.map_object, route_s, route, self.vehicle_sampler, eval_range)
        all_other_vehicle_list.extend(vehicle_list)

        # 3) Sample vehicles on lanes which belongs to the road of occluder and ego
        #    but have reverse direction
        route_list = [
            [[25, 1, 'end'], [16, 1, 'end']],
            [[25, 2, 'end'], [16, 2, 'end']]
        ]
        for route in route_list:
            vehicle_list = scene_utils.sample_vehicles_along_route(self.map_object, 0.0, route, self.vehicle_sampler)
            all_other_vehicle_list.extend(vehicle_list)

        # 4.1) Sample vehicles that are passing junction
        route_list = [
            [[39, -2, 'start'], [1615, -1, 'start'], [0, -2, 'start']],  # go straight
            [[39, -1, 'start'], [1623, -1, 'start'], [0, -1, 'start']],  # go straight
        ]
        for route in route_list:
            vehicle_list = scene_utils.sample_vehicles_along_route(self.map_object, 0.0, route, self.vehicle_sampler)
            all_other_vehicle_list.extend(vehicle_list)

        # 4.2) Sample vehicles that are passing junction by turing left
        route = [[1715, -1, 'start'], [26, -1, 'start'], [29, 1, 'end']]    # turn left
        init_s = self.map_object.getRoad(1715).length
        vehicle_list = scene_utils.sample_vehicles_along_route(self.map_object, init_s, route, self.vehicle_sampler)
        all_other_vehicle_list.extend(vehicle_list)

        # 4.3) Sample vehicles that are waiting for the red light
        #  TODO customize vehicle sampler for waiting for red light
        route_list = [
            [[0, 1, 'end']],
            [[0, 2, 'end']],
            [[29, -1, 'start'], [26, 1, 'end']],
            [[29, -2, 'start'], [26, 2, 'end']]
        ]
        vehicle_center_to_stop_line = 3.0 # meters
        stop_line_pos_list = [
            [0, 1, vehicle_center_to_stop_line],  # [road_id, lane_id, s],
            [0, 2, vehicle_center_to_stop_line],
            [26, 1, vehicle_center_to_stop_line],
            [26, 2, vehicle_center_to_stop_line]
        ]
        
        for route,stop_line_pos in zip(route_list, stop_line_pos_list):
            # sample vehicle at stop line
            veh = self.random_sample_vehicle_at(stop_line_pos[0], stop_line_pos[1], stop_line_pos[2])
            
            # sample vehicles in the following
            init_s = route_utils.calc_route_s_from_waypoint(self.map_object, route, veh["waypoint"])
            init_s -= self.atm.get_actor_type(veh["type_id"]).x
            vehicle_list = scene_utils.sample_vehicles_against_route(self.map_object, init_s, route, self.vehicle_sampler)
            vehicle_list.append(veh)
            all_other_vehicle_list.extend(vehicle_list)

        # 5) Filter vehicles by range
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
    map_path = "/home/lrs/scripts/CoRiskyScene/coriskyscene/data_collection/data/map_opendrive/Town05.xodr"
    rushout_gen = JuncitonRushoutSceneGenerator("juncrush", map_path)
    rushout_gen.run()
