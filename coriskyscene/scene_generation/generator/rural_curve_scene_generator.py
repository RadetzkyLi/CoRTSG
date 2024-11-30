#!/usr/bin/env python
# -*- coding: utf-8 -*-
'''
@File    :   rural_curve_scene_generator.py
@Date    :   2024-01-25
@Author  :   Rongsong Li <rongsong.li@qq.com>
@Version :   1.0
@Desc    :   Generate traffic scenes for rural curve
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


class RuralCurveSceneGenerator(BaseSceneSequentialGenerator):
    """
    
    Scene Desc
    ----------
    In rural curvy road segment with single but bidirectional lane (
    i.e., without road center line mark), occluded by hill/trees (B), 
    ego (A) is moving forward and collides head on head with another vehilce 
    (C) which is aslo moving forward.

        -------------------+
           C ---->          \ 
        --------------+      \ 
              🌳🌳B   \      \ 
                        \  ↖   \ 
                         \   A  \ 
    
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
            return my_sampler.StaticOccluderTypeSampler()
        elif _type == "occludee":
            return my_sampler.OccludeeTypeSampler(target_main_classes=['vehicle'])
        else:
            raise ValueError("Unexpected `_type`(='{0}')".format(_type))
        
    def get_ego_waypoint_list(self, **kwargs):
        """
        The ego is on road segment
        """
        longitudinal_resolution = self.config.longitudinal_resolution  
        route = [[26, 1, 'end'], [25, 1, 'end']]

        wp_list = route_utils.sample_waypoints_along_route(self.map_object, route, 0.0, longitudinal_resolution)

        return wp_list
    
    def get_occluder_waypoint_list(self, ego, **kwargs):
        """
        The occluder is fixed at one position.
        """
        road_id,lane_id = 26,-3
        road = self.map_object.getRoad(road_id)

        wp_list = [road.calcWaypoint(0, lane_id, 0.0)]

        return wp_list
    
    def get_occludee_waypoint_list(self, ego, occluder, **kwargs):
        """
        The occludee is on road segment.
        """
        eval_range = self.config.eval_range
        longitudinal_resolution = self.config.longitudinal_resolution
        route = [[20, 1, 'end'], [497, 1, 'end']]
        
        if not hasattr(self, "tmp_occludee_wp_list"):
            # precompute to save time
            init_s = 256.0 - eval_range 
            # init_s = 0.0
            wp_list = route_utils.sample_waypoints_along_route(self.map_object, route, init_s, longitudinal_resolution)
            print("total occludee wp", len(wp_list))
            self.tmp_occludee_wp_list = wp_list
        else:
            wp_list = self.tmp_occludee_wp_list

        # filter by range
        wp_list = [wp for wp in wp_list if trasc.calc_distance_between_waypoints(wp, ego.waypoint)<=eval_range]

        return wp_list
    
    def sample_others_ar(self, ego, occluder, occludee, seed, eval_range):
        """Sample other road users in auto regressive manner.
        TODO get route automatically.
        
        """
        self.oim.reset_to(3)
        random.seed(seed)
        self.vehicle_sampler.set_seed(seed)

        all_other_vehicle_list = []
        minimal_dist_to_leading_vehicle = 10.0  # front bumper to rear bumper
        
        # 1) sample ego's following vehicles
        route = [[26, 1, 'end'], [25, 1, 'end']]
        ego_rear_route_s = route_utils.calc_route_s_for_actor(self.map_object, route, ego, 'rear')
        init_s = ego_rear_route_s - minimal_dist_to_leading_vehicle
        if init_s > 0:
            vehicle_list = scene_utils.sample_vehicles_against_route(self.map_object, init_s, route, self.vehicle_sampler)
            all_other_vehicle_list.extend(vehicle_list)
        
        # 2) sample occludee's following vehicles
        route = [[20, 1, 'end'], [497, 1, 'end']]
        occludee_rear_route_s = route_utils.calc_route_s_for_actor(self.map_object, route, occludee, 'rear')
        init_s = occludee_rear_route_s - minimal_dist_to_leading_vehicle
        if init_s > 0:
            vehicle_list = scene_utils.sample_vehicles_against_route(self.map_object, init_s, route, self.vehicle_sampler)
            all_other_vehicle_list.extend(vehicle_list)

        # 3) Filter vehicles by range
        center_actor = {
            "type_id": ego.type_id,
            "waypoint": ego.waypoint
        }
        all_other_vehicle_list = self.filter_actors_by_range(all_other_vehicle_list, center_actor, eval_range)

        # 4) convert to CarlaActor
        all_other_vehicle_list = [
            self.get_actor_from_actor_type(self.atm, actor["type_id"], self.oim, actor["waypoint"]) 
            for actor in all_other_vehicle_list
        ]
        
        return all_other_vehicle_list
    
    def check_and_construct_scene(self, ego, occluder, occludee, npc_list, perception_range=70, occluder_as_cav=True, ignore_none_valid_cav=False):
        """Save the scene if occludee is of 'car' """
        return self.check_and_construct_scene_v2(ego, occluder, occludee, npc_list, perception_range, occluder_as_cav, ignore_none_valid_cav)
    
class RuralCurveSceneGeneratorL2(RuralCurveSceneGenerator):
    """
    Rural curve scene generator in another location.
    """
    def __init__(self, name: str, map_path=None) -> None:
        super().__init__(name, map_path)

    def get_type_sampler_for(self, _type):
        """"Get actor type sampler"""
        if _type == "ego":
            return my_sampler.EgoTypeSampler()
        elif _type == "occluder":
            return my_sampler.StaticOccluderTypeSampler(['virtual.static.0001'])
        elif _type == "occludee":
            return my_sampler.OccludeeTypeSampler(target_main_classes=['vehicle'])
        else:
            raise ValueError("Unexpected `_type`(='{0}')".format(_type))

    def get_ego_waypoint_list(self, **kwargs):
        """
        The ego is on road segment
        """
        longitudinal_resolution = self.config.longitudinal_resolution
        route = [[20, -1, 'start']]
        init_s = 125.0
        wp_list = route_utils.sample_waypoints_against_route(self.map_object, route, init_s, longitudinal_resolution)

        return wp_list
    
    def get_occluder_waypoint_list(self, ego, **kwargs):
        """
        The occluder is a virtual static object.
        """
        road_id,lane_id = 20,2
        s_pos = 90.0
        road = self.map_object.getRoad(road_id)
        wp_list = [road.calcWaypoint(0, lane_id, s_pos)]

        return wp_list
    
    def get_occludee_waypoint_list(self, ego, occluder, **kwargs):
        """
        The occludee is also on road segment.
        """
        eval_range = self.config.eval_range
        longitudinal_resolution = self.config.longitudinal_resolution

        route = [[20, 1, 'end']]
        init_s = 256.0 - 125.0
        wp_list = route_utils.sample_waypoints_against_route(self.map_object,  route, init_s, longitudinal_resolution)

        # filter by range
        wp_list = [wp for wp in wp_list if trasc.calc_distance_between_waypoints(wp, ego.waypoint)<eval_range]

        return wp_list
    
    def sample_others_ar(self, ego, occluder, occludee, seed, eval_range):
        """Sample other road users in auto regressive manner.
        TODO get route automatically.
        
        """
        self.oim.reset_to(3)
        random.seed(seed)
        self.vehicle_sampler.set_seed(seed)

        all_other_vehicle_list = []
        
        # 1) sample ego's following vehicles
        route = [[20, -1, 'start']]
        ego_rear_route_s = route_utils.calc_route_s_for_actor(self.map_object, route, ego, 'rear')
        init_s = ego_rear_route_s
        if init_s > 0:
            vehicle_list = scene_utils.sample_vehicles_against_route(self.map_object, init_s, route, self.vehicle_sampler)
            all_other_vehicle_list.extend(vehicle_list)
        
        # 2) sample occludee's following vehicles
        route = [[20, 1, 'end']]
        occludee_rear_route_s = route_utils.calc_route_s_for_actor(self.map_object, route, occludee, 'rear')
        init_s = occludee_rear_route_s
        if init_s > 0:
            vehicle_list = scene_utils.sample_vehicles_against_route(self.map_object, init_s, route, self.vehicle_sampler)
            all_other_vehicle_list.extend(vehicle_list)

        # 3) Filter vehicles by range
        center_actor = {
            "type_id": ego.type_id,
            "waypoint": ego.waypoint
        }
        all_other_vehicle_list = self.filter_actors_by_range(all_other_vehicle_list, center_actor, eval_range)

        # 4) convert to CarlaActor
        all_other_vehicle_list = [
            self.get_actor_from_actor_type(self.atm, actor["type_id"], self.oim, actor["waypoint"]) 
            for actor in all_other_vehicle_list
        ]
        
        return all_other_vehicle_list
    

if __name__ == '__main__':
    # generate scenes
    logging.basicConfig(format='%(levelname)s: %(message)s', level=logging.INFO)

    # replace as your local path 
    map_path = "/home/lrs/scripts/CoRiskyScene/coriskyscene/data_collection/data/map_opendrive/Town07.xodr"

    # gen = RuralCurveSceneGenerator("ruralcurve", map_path)
    gen = RuralCurveSceneGeneratorL2("ruralcurve_l2", map_path)
    gen.run()
