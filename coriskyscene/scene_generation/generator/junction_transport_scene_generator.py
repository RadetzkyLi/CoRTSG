#!/usr/bin/env python
# -*- coding: utf-8 -*-
'''
@File    :   junction_transport_scene_generator.py
@Date    :   2024-01-27
@Author  :   Rongsong Li <rongsong.li@qq.com>
@Version :   1.0
@Desc    :   Generate traffic scenes in which vehicle in transport is occluder.
'''

import os
import sys
import logging
import random
from lxml import etree
import itertools
import numpy as np

from coriskyscene.opendriveparser import parser as odr_parser
from coriskyscene.scene_generation import traffic_scene as trasc
from coriskyscene.scene_generation.generator.base_scene_sequential_generator \
    import BaseSceneSequentialGenerator
from coriskyscene.scene_generation.samplers import my_sampler
from coriskyscene.scene_generation.utils import scene_utils, route_utils


# ===============================================================================
# - Base Juction Scene Generator
# ===============================================================================
class BaseJunctionOcclusionSceneGenerator(BaseSceneSequentialGenerator):
    """
    Base class for traffic scenes in junction.
    """
    def __init__(self, name: str, map_path:str=None) -> None:
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
    

    @staticmethod
    def is_waypoint_in_junction(map_object, waypoint):
        """
        Returns True if the given waypoint lies in a junction
        """
        road_id = waypoint.road_id
        road = map_object.getRoad(road_id)
        if road.junction is None:
            return False
        return True
    
    @staticmethod
    def check_lane_uniqueness_for_routes(route_list):
        """
        Ensure a lane just appear in route for once.

        Parameters
        ----------
        route_list : list

        Returns
        -------
        flag : bool
            Returns False if some lane apprears more than once.
        """
        all_edges = list(itertools.chain(*route_list))
        # structure: {"road_id": [lane_id1, lane_id2, ...]}
        road_lane_dict = {edge[0]: [] for edge in all_edges}
        for edge in all_edges:
            road_lane_dict[edge[0]].append(edge[1])

        # check lane frequency for each road
        len_diff = {k: len(v)-len(set(v)) for k,v in road_lane_dict.items()}
        # some lane appear more than once
        if np.sum(np.array(list(len_diff.values()))>0) > 0:
            return False
        return True
    
    @staticmethod
    def has_collision_between_actor_group(actor_list, other_actor_list):
        """Check collision between two groups of actors. The two group
        have empty intersection.
        TODO : check intersection
        
        Returns
        -------
        flag : bool
            Retrurns True if there is collision
        """
        for actor,other in itertools.product(actor_list, other_actor_list):
            if actor.bounding_box.intersects(other.bounding_box):
                return True
        return False
    
    @staticmethod
    def has_collision_in_junction_between(map_object, actor_list, other_list):
        actor_list = [actor for actor in actor_list 
                               if map_object.getRoad(actor.waypoint.road_id).junction is not None]
        other_list = [actor for actor in other_list
                               if map_object.getRoad(actor.waypoint.road_id).junction is not None]
        for actor,other in itertools.product(actor_list, other_list):
            if actor.bounding_box.intersects(other.bounding_box):
                return True
        return False
    
    def has_collision_among_key_actors(self, ego, occluder, occludee):
        """Returns True if collision happens"""
        return self.has_collision_among_actors([ego, occluder, occludee])
    
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
    
    def sample_vehicles_in_routes(self, route_list, existing_actor_list):
        """Sample vehicles for given routes.
        We assume:
            1. car-following for all routes.
            2. the routes have no common lanes.
        So, the collision can only happen in intersection

        Parameters
        ----------
        route_list : list
            Route will be visited by its order in list.

        existing_actor_list : list
            A list of CarlaAcotor.

        Returns
        -------
        vehicle_list : list

        """
        # TODO check route's connectivity
        
        # check lane uniqueness
        if not self.check_lane_uniqueness_for_routes(route_list):
            raise ValueError("There are lanes appearing twice or more!")
        
        vehicle_list = []
        # stop sampling after `num_max_tries` consecutive failures from beginning
        num_max_tries = 3

        # go through all routes
        for route in route_list:
            # sample from route's start
            current_actor = None
            route_len_list = route_utils.get_route_len_list(self.map_object, route)
            num_fail = 0
            while True:
                if num_fail>= num_max_tries:
                    break

                vehicle = scene_utils.sample_leading_vehicle_for_actor(self.map_object, route, self.vehicle_sampler, current_actor, route_len_list)
                if vehicle is None:
                    break
                
                # check collision
                vehicle = self.get_actor_from_actor_type(self.atm, vehicle["type_id"], self.oim, vehicle["waypoint"])
                if self.has_collision_between_actor_group(existing_actor_list+vehicle_list, [vehicle]):
                    num_fail += 1
                    continue

                vehicle_list.append(vehicle)
                num_fail = 0
                current_actor = vehicle

        return vehicle_list


# =========================================================================================
# - occluder is moving forward in reverse direction
# =========================================================================================
class JunctionTransportReverseDirectionSceneGenerator(BaseJunctionOcclusionSceneGenerator):
    """

    Scene Desc
    ----------
    In a four lag junction, ego (A) is trying to turn left, occluded by another vehicle (B)
    that is moving straight on adjacent lane of reverse direction, ego collides
    with the third vehicle (C) that is moving straight on perpendicular lane of ego.

                        |   |   |     
    --------------------+       +-------------
                      <--- A'       
    ---------------------       --------------
           (C) ---->
    --------------------+       +-------------
                        | ↓ |   |
                        | B | ↑ | 
                        |   | A |
    
    """
    def __init__(self, name: str, map_path=None) -> None:
        super().__init__(name, map_path)

    def get_ego_waypoint_list(self, **kwargs):
        """
        The ego is going to turn left
        """
        longitudinal_resolution = self.config.longitudinal_resolution
        route = [[2, 1, 'end'], [1114, 1, 'end']]
        init_s = self.map_object.getRoad(2).length*0.5

        wp_list = route_utils.sample_waypoints_along_route(self.map_object, route, init_s, longitudinal_resolution)
        return wp_list

    def get_occluder_waypoint_list(self, ego, **kwargs):
        """
        The occluder is moving straight
        """
        longitudinal_resolution = self.config.longitudinal_resolution
        eval_range = self.config.eval_range

        # route = [[1072, -1, 'start'], [2, -1, 'start']]
        # length = self.map_object.getRoad(1072).length
        # wp_list = route_utils.sample_waypoints_along_route(self.map_object, route, 0.0, longitudinal_resolution)
        # wp_list = route_utils.sample_waypoints_against_route(self.map_object, route, length+10.0, longitudinal_resolution)
        wp_list = [self.map_object.getRoad(2).calcWaypoint(0, -1, 0.0)]

        # filter points out of range
        wp_list = [wp for wp in wp_list if trasc.calc_distance_between_waypoints(wp, ego.waypoint)<eval_range]

        return wp_list
        
    def get_occludee_waypoint_list(self, ego, occluder, **kwargs):
        """
        The occludee is moving straight
        """
        longitudinal_resolution = self.config.longitudinal_resolution
        eval_range = self.config.eval_range
        
        route = [[49, -1, 'start'], [1046, -1, 'start']]
        wp_list = route_utils.sample_waypoints_along_route(self.map_object, route, 0.0, longitudinal_resolution)

        # filter points out of range
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

        # 1) Sample ego's leading/following vehicles
        route = [[2, 1, 'end'], [1114, 1, 'end'], [49, 1, 'end'], [5, 1, 'end']]
        length1 = self.map_object.getRoad(2).length
        length2 = self.map_object.getRoad(1114).length
        
        # 1.1) lv
        minimal_dist_to_leading_vehicle = 10  # ego's front bumper to lv's rear when turn left
        ego_front_route_s = route_utils.calc_route_s_for_actor(self.map_object, route, ego, 'front')
        init_s = max(ego_front_route_s+minimal_dist_to_leading_vehicle, length1+length2*0.75)
        vehicle_list = scene_utils.sample_vehicles_along_route(self.map_object, init_s, route, self.vehicle_sampler)
        all_other_vehicle_list.extend(vehicle_list)

        # 1.2) fv
        ego_rear_route_s = route_utils.calc_route_s_for_actor(self.map_object, route, ego, 'rear')
        init_s = min(ego_rear_route_s, length1)
        if init_s>0:
            vehicle_list = scene_utils.sample_vehicles_against_route(self.map_object, init_s, route, self.vehicle_sampler)
            all_other_vehicle_list.extend(vehicle_list)

        # 2) Sample occluder's leading/following vehicles
        route = [[1, -1, 'start'], [1072, -1, 'start'], [2, -1, 'start']]
        length1 = self.map_object.getRoad(1).length
        length2 = self.map_object.getRoad(1072).length
        
        # 2.1) lv
        occluder_front_route_s = route_utils.calc_route_s_for_actor(self.map_object, route, occluder, 'front')
        init_s = max(occluder_front_route_s, length1+length2)
        vehicle_list = scene_utils.sample_vehicles_along_route(self.map_object, init_s, route, self.vehicle_sampler)
        all_other_vehicle_list.extend(vehicle_list)

        # 2.2) fv: hasn't enter the junction
        occluder_rear_route_s = route_utils.calc_route_s_for_actor(self.map_object, route, occluder, 'rear')
        init_s = min(occluder_rear_route_s, length1+length2*0.25)
        if init_s>0:
            vehicle_list = scene_utils.sample_vehicles_against_route(self.map_object, init_s, route, self.vehicle_sampler)
            all_other_vehicle_list.extend(vehicle_list)

        # 3) Sample occludee's leading/following vehicles
        route = [[5, -1, 'start'], [49, -1, 'start'], [1046, -1, 'start'], [50, -1, 'start'], [52, 1, 'end']]
        route_len_list = route_utils.get_route_len_list(self.map_object, route)
        
        # 3.1) lv
        occludee_front_route_s = route_utils.calc_route_s_for_actor(self.map_object, route, occludee, 'front')
        init_s = max(occludee_front_route_s, route_len_list[3])
        vehicle_list = scene_utils.sample_vehicles_along_route(self.map_object, init_s, route, self.vehicle_sampler)
        all_other_vehicle_list.extend(vehicle_list)

        # 3.2) fv
        occludee_rear_route_s = route_utils.calc_route_s_for_actor(self.map_object, route, occludee, 'rear')
        init_s = occludee_rear_route_s
        if init_s>0:
            vehicle_list = scene_utils.sample_vehicles_against_route(self.map_object, init_s, route, self.vehicle_sampler)
            all_other_vehicle_list.extend(vehicle_list)


        # 4) Filter vehicles by range
        # center_actor = {
        #     "type_id": ego.type_id,
        #     "waypoint": ego.waypoint
        # }
        # all_other_vehicle_list = self.filter_actors_by_range(all_other_vehicle_list, center_actor, eval_range)

        # 5) convert to CarlaActor
        all_other_vehicle_list = [
            self.get_actor_from_actor_type(self.atm, actor["type_id"], self.oim, actor["waypoint"]) 
            for actor in all_other_vehicle_list
        ]

        # 6) Sample others
        route_list = [
            # a) oncoming road 2
            # turn right
            [[2, 2, 'end'], [1000, 1, 'end'], [50, -2, 'start'], [52, 2, 'end']],   
            # b) oncoming road 49
            # turn right
            [[5, -2, 'start'], [1121, -1, 'start'], [2, -2, 'start']],
            # turn left
            # [[1094, -1, 'start'], [1, 1, 'end']],
            # c) oncoming road 1
            # forward but far from junction
            [[1, -2, 'start'], [1100, -1, 'start']],
            # d) oncoming road 50
            # forward
            [[52, -2, 'start'], [50, 2, 'end'], [1047, 2, 'end'], [49, 2, 'end'], [5, 2, 'end']],
            # forward but far from juction
            [[52, -1, 'start'], [50, -1, 'start']],
            # e) outgoing roads
            [[1, 1, 'end']],
            [[1, 2, 'end']]

        ]
        vehicle_list = self.sample_vehicles_in_routes(route_list, [ego, occluder, occludee]+all_other_vehicle_list)

        all_other_vehicle_list.extend(vehicle_list)

        return all_other_vehicle_list


# ===========================================================================================
# - occluder is turning left
# ===========================================================================================
class JunctionTransportTurnLeftSceneGenerator(BaseJunctionOcclusionSceneGenerator):
    """
    Scene Desc
    ----------
    In a four lag junction, ego (A) is trying to turn left, occluded by another vehicle (B)
    that is turning left, ego collides with the third vehicle (C) that is moving 
    straight in ego's perpendicular direction.

                        |   |   |     
    --------------------+       +-------------
                  <--- A'     <--- C  
    ---------------------    ↘B --------------
          
    --------------------+     ↑ +-------------
                        |   | A |
                        |   |   | 
                        |   |   |
    
    """
    def __init__(self, name: str, map_path: str = None) -> None:
        super().__init__(name, map_path)


    def get_conflict_point(self):
        """

        Returns
        -------
        conflict_point : Waypoint
            The point at which A and C collides.
        """
        road_id,lane_id,s_pos = 1046,-1,0.0
        conflict_point = self.map_object.getRoad(road_id).calcWaypoint(0, lane_id, s_pos)

        return conflict_point

    def get_ego_waypoint_list(self, **kwargs):
        """
        The ego is turning left but shouldn't have finished turning
        """
        longitudinal_resolution = self.config.longitudinal_resolution
        route = [[1114, 1, 'end']]
        length = self.map_object.getRoad(1114).length
        init_s = length * 0.5

        # wp_list = route_utils.sample_waypoints_along_route(self.map_object, route, 0.0, longitudinal_resolution)
        wp_list = route_utils.sample_waypoints_against_route(self.map_object, route, init_s, longitudinal_resolution)
        return wp_list
    
    def get_occluder_waypoint_list(self, ego, **kwargs):
        """
        The occluder is turning left
        """
        longitudinal_resolution = self.config.longitudinal_resolution
        eval_range = self.config.eval_range

        route = [[987, -1, 'start']]
        wp_list = route_utils.sample_waypoints_along_route(self.map_object, route, 0.0, longitudinal_resolution)
       
        # filter by range
        wp_list = [wp for wp in wp_list if trasc.calc_distance_between_waypoints(wp, ego.waypoint)<=eval_range]
        return wp_list
    
    def get_occludee_waypoint_list(self, ego, occluder, **kwargs):
        """
        The occludee is moving straight
        """
        longitudinal_resolution = self.config.longitudinal_resolution
        eval_range = self.config.eval_range

        route = [[50, 1, 'end'], [1047, 1, 'end']]
        wp_list = route_utils.sample_waypoints_along_route(self.map_object, route, 0.0, longitudinal_resolution)

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

        # 1) Sample ego's leading/following vehicles
        route = [[2, 1, 'end'], [1114, 1, 'end'], [49, 1, 'end'], [5, 1, 'end']]
        length1 = self.map_object.getRoad(2).length
        length2 = self.map_object.getRoad(1114).length
        length3 = self.map_object.getRoad(49).length
        
        # 1.1) lv: should out of juction
        minimal_dist_to_leading_vehicle = 10  # ego's front bumper to lv's rear when turn left
        ego_front_route_s = route_utils.calc_route_s_for_actor(self.map_object, route, ego, 'front')
        init_s = max(ego_front_route_s+minimal_dist_to_leading_vehicle, length1+length2+length3)
        vehicle_list = scene_utils.sample_vehicles_along_route(self.map_object, init_s, route, self.vehicle_sampler)
        all_other_vehicle_list.extend(vehicle_list)

        # 1.2) fv
        ego_rear_route_s = route_utils.calc_route_s_for_actor(self.map_object, route, ego, 'rear')
        init_s = ego_rear_route_s
        if init_s>0:
            vehicle_list = scene_utils.sample_vehicles_against_route(self.map_object, init_s, route, self.vehicle_sampler)
            all_other_vehicle_list.extend(vehicle_list)

        # 2) Sample occluder's leading/following vehicles: should out of junction
        route = [[1, -1, 'start'], [987, -1, 'start'], [50, -1, 'start'], [52, 1, 'end']]
        length1 = self.map_object.getRoad(1).length
        length2 = self.map_object.getRoad(987).length

        # 2.1) lv
        occluder_front_route_s = route_utils.calc_route_s_for_actor(self.map_object, route, occluder, 'front')
        init_s = max(occluder_front_route_s, length1+length2)
        vehicle_list = scene_utils.sample_vehicles_along_route(self.map_object, init_s, route, self.vehicle_sampler)
        all_other_vehicle_list.extend(vehicle_list)

        # 2.2) fv
        occluder_rear_route_s = route_utils.calc_route_s_for_actor(self.map_object, route, occluder, 'rear')
        init_s = min(occluder_rear_route_s, length1)
        vehicle_list = scene_utils.sample_vehicles_against_route(self.map_object, init_s, route, self.vehicle_sampler)
        all_other_vehicle_list.extend(vehicle_list)

        # 3) Sample occludee's following vehicles
        route = [[52, -1, 'start'], [50, 1, 'end'], [1047, 1, 'end']]
        occludee_rear_route_s = route_utils.calc_route_s_for_actor(self.map_object, route, occludee, 'rear')
        vehicle_list = scene_utils.sample_vehicles_against_route(self.map_object, occludee_rear_route_s, route, self.vehicle_sampler)
        all_other_vehicle_list.extend(vehicle_list)

        # 5) convert to CarlaActor
        all_other_vehicle_list = [
            self.get_actor_from_actor_type(self.atm, actor["type_id"], self.oim, actor["waypoint"]) 
            for actor in all_other_vehicle_list
        ]

        # 6) Sample others
        route_list = [
            # a) oncoming road 2
            # turn right
            [[2, 2, 'end'], [1000, 1, 'end'], [50, -2, 'start'], [52, 2, 'end']],
            # b) oncoming road 49
            # turn right
            [[5, -2, 'start'], [49, -2, 'start'], [1121, -1, 'start'], [2, -2, 'start']],
            # forward but far from junction
            [[5, -1, 'start'], [49, -1, 'start']],
            # c) oncoming road 1
            # turn right
            [[1, -2, 'start'], [1100, -1, 'start'], [49, 2, 'end'], [5, 2, 'end']],
            # d) oncoming road 50
            # turn right
            [[52, -2, 'start'], [981, 1, 'end'], [1, 2, 'end']],
            # e) outgoing road
            [[1, 1, 'end']],
            [[2, -1, 'start']]
        ]
        vehicle_list = self.sample_vehicles_in_routes(route_list, [ego, occluder, occludee]+all_other_vehicle_list)

        all_other_vehicle_list.extend(vehicle_list)

        return all_other_vehicle_list


# ===================================================================================
# - occluder is moving forward in same direction
# ===================================================================================
class JucntionTransportSameDirectionSceneGenerator(BaseJunctionOcclusionSceneGenerator):
    """
    
    In a four lag junction, ego (A) is trying to turn left, occluded by another vehicle (B)
    that is moving forward, ego collides with the third vehicle (C) that is moving 
    straight in reverse direction.
                        
                        | C |   |
                        | ↓ |   |     
    --------------------+       +-------------
                  <--- A'     
    ---------------------    ↑ B -------------
          
    --------------------+     ↑ +-------------
                        |   | A |
                        |   |   | 
                        |   |   |
    
    """
    def __init__(self, name: str, map_path: str = None) -> None:
        super().__init__(name, map_path)

    def get_ego_waypoint_list(self, **kwargs):
        """
        The ego is in junction
        """
        longitudinal_resolution = self.config.longitudinal_resolution
        route = [[1114, 1, 'end']]

        wp_list = route_utils.sample_waypoints_along_route(self.map_object, route, 0.0, longitudinal_resolution)
        return wp_list
    
    def get_occluder_waypoint_list(self, ego, **kwargs):
        """
        The occluder is in front of ego and in junction.
        """
        longitudinal_resolution = self.config.longitudinal_resolution
        eval_range = self.config.eval_range
        minimal_dist_to_ego = 10 + 5  # center to center

        route = [[2, 1, 'end'], [1071, 1, 'end']]
        route = [[1071, 1, 'end']]
        wp_list = route_utils.sample_waypoints_along_route(self.map_object, route, 0.0, longitudinal_resolution)

        # filter by range
        wp_list = [wp for wp in wp_list if trasc.calc_distance_between_waypoints(wp, ego.waypoint)>minimal_dist_to_ego]
        wp_list = [wp for wp in wp_list if trasc.calc_distance_between_waypoints(wp, ego.waypoint)<=eval_range]

        return wp_list
    
    def get_occludee_waypoint_list(self, ego, occluder, **kwargs):
        """
        The occludee is moving forward.
        """
        longitudinal_resolution = self.config.longitudinal_resolution
        eval_range = self.config.eval_range

        route = [[1, -1, 'start'], [1072, -1, 'start']]
        init_s = self.map_object.getRoad(1).length*0.5
        wp_list = route_utils.sample_waypoints_along_route(self.map_object, route, init_s, longitudinal_resolution)

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

        
        # 1) Sample ego's leading/following vehicles
        route = [[2, 1, 'end'], [1114, 1, 'end'], [49, 1, 'end'], [5, 1, 'end']]
        length1 = self.map_object.getRoad(2).length
        length2 = self.map_object.getRoad(1114).length
        length3 = self.map_object.getRoad(49).length
        
        # 1.1) lv: should out of juction
        minimal_dist_to_leading_vehicle = 10  # ego's front bumper to lv's rear when turn left
        ego_front_route_s = route_utils.calc_route_s_for_actor(self.map_object, route, ego, 'front')
        init_s = max(ego_front_route_s+minimal_dist_to_leading_vehicle, length1+length2+length3)
        vehicle_list = scene_utils.sample_vehicles_along_route(self.map_object, init_s, route, self.vehicle_sampler)
        all_other_vehicle_list.extend(vehicle_list)

        # 1.2) fv
        ego_rear_route_s = route_utils.calc_route_s_for_actor(self.map_object, route, ego, 'rear')
        init_s = ego_rear_route_s
        if init_s>0:
            vehicle_list = scene_utils.sample_vehicles_against_route(self.map_object, init_s, route, self.vehicle_sampler)
            all_other_vehicle_list.extend(vehicle_list)

        # 2) Sample occluder's leading vehicles
        route = [[1071, 1, 'end'], [1, 1, 'end']]
        occluder_front_route_s = route_utils.calc_route_s_for_actor(self.map_object, route, occluder, 'front')
        vehicle_list = scene_utils.sample_vehicles_along_route(self.map_object, occluder_front_route_s, route, self.vehicle_sampler)
        all_other_vehicle_list.extend(vehicle_list)

        # 3) Sample occludee's leading/following vehicles: should out of junction
        route = [[1, -1, 'start'], [1072, -1, 'start'], [2, -1, 'start']]
        length1 = self.map_object.getRoad(1).length
        length2 = self.map_object.getRoad(1072).length

        # 3.1) lv
        occludee_front_route_s = route_utils.calc_route_s_for_actor(self.map_object, route, occludee, 'front')
        init_s = max(occludee_front_route_s, length1+length2)
        vehicle_list = scene_utils.sample_vehicles_along_route(self.map_object, init_s, route, self.vehicle_sampler)
        all_other_vehicle_list.extend(vehicle_list)

        # 3.2) fv
        occludee_rear_route_s = route_utils.calc_route_s_for_actor(self.map_object, route, occludee, 'rear')
        init_s = min(occludee_rear_route_s, length1)
        vehicle_list = scene_utils.sample_vehicles_against_route(self.map_object, init_s, route, self.vehicle_sampler)
        all_other_vehicle_list.extend(vehicle_list)

        
        # 5) convert to CarlaActor
        all_other_vehicle_list = [
            self.get_actor_from_actor_type(self.atm, actor["type_id"], self.oim, actor["waypoint"]) 
            for actor in all_other_vehicle_list
        ]

        # 6) Sample others
        route_list = [
            # a) oncoming road 2
            # forward
            [[2, 2, 'end'], [1071, 2, 'end'], [1, 2, 'end']],
            # b) oncoming road 49
            # turn right
            [[5, -2, 'start'], [49, -2, 'start'], [1121, -1, 'start'], [2, -2, 'start']],
            # c) oncoming road 1
            # turn right
            [[1, -2, 'start'], [1100, -1, 'start'], [49, 2, 'end'], [5, 2, 'end']],
            # d) oncoming road 50
            # forward but out of junction
            [[52, -1, 'start'], [50, 1, 'end']],
            [[52, -2, 'start'], [50, 2, 'end']],
            # e) outgoing road
            [[50, -1, 'start'], [52, 1, 'end']],
            [[50, -2, 'start'], [52, 2, 'end']]
        ]
        vehicle_list = self.sample_vehicles_in_routes(route_list, [ego, occluder, occludee]+all_other_vehicle_list)

        all_other_vehicle_list.extend(vehicle_list)

        return all_other_vehicle_list

if __name__ == '__main__':
    # generate scenes
    logging.basicConfig(format='%(levelname)s: %(message)s', level=logging.INFO)

    # replace as your local path
    map_path = "/home/lrs/scripts/CoRiskyScene/coriskyscene/data_collection/data/map_opendrive/Town05.xodr"

    # gen = JunctionTransportReverseDirectionSceneGenerator("juncreverse", map_path)

    # gen = JunctionTransportTurnLeftSceneGenerator("juncturn", map_path)

    gen = JucntionTransportSameDirectionSceneGenerator("juncsame", map_path)

    gen.run()
