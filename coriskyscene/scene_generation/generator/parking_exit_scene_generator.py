#!/usr/bin/env python
# -*- coding: utf-8 -*-
'''
@File    :   parking_exit_scene_generator.py
@Date    :   2024-01-26
@Author  :   Rongsong Li <rongsong.li@qq.com>
@Version :   1.0
@Desc    :   Generate traffic scenes near parking
'''

import os
import time
import logging
import random
import copy
import itertools
from lxml import etree

from coriskyscene.opendriveparser import parser as odr_parser
from coriskyscene.scene_generation import traffic_scene as trasc
from coriskyscene.scene_generation.generator.base_scene_sequential_generator \
    import BaseSceneSequentialGenerator
from coriskyscene.scene_generation.samplers import my_sampler
from coriskyscene.scene_generation.utils import scene_utils, route_utils


# ==================================================================
# - occluder group sampler
# ==================================================================
class OccluderGroupSampler(my_sampler.BaseActorSampler):
    """
    Sample multiple occluders at one time.
    """
    def __init__(self, type_sampler, waypoint_group_list, waypoint_sampler=None) -> None:
        super().__init__(type_sampler, waypoint_sampler)

        self.waypoint_group_list = waypoint_group_list
        self._length = int(len(self.type_sampler)) * len(self.waypoint_group_list)

        self.type_list = self.type_sampler.get_all_types()
        self.sample_generator = itertools.product(self.type_list, self.waypoint_group_list)

    def __len__(self):
        return self._length
    
    def get_sample_sequentially(self):
        """
        
        Returns
        -------
        _type : str
            Ego's type id

        waypoint : Waypoint
        """
        try:
            _type,waypoint_group = next(self.sample_generator)
        except StopIteration:
            _type,waypoint_group = None,None
        finally:
            return _type,waypoint_group


class ParkingExitSceneGenerator(BaseSceneSequentialGenerator):
    """
    Scene Desc
    ----------
    Near exit of a parking lot, ego (A) is moving straight, occluded by
    building/vegetation (B), ego collides with another vehicle (C) that 
    is exiting parking lot.

        -------------------------------------------
                                        <---- npc
        -------------------------------------------
                 A ---->
        ------------------------+     +------------
                        🌳🌳B  |  ↑  |  🌳🌳B
                                |  C  |
                             (parking lot)
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
            return my_sampler.StaticOccluderTypeSampler(static_types=['virtual.static.0003'])
        elif _type == "occludee":
            return my_sampler.OccludeeTypeSampler(target_main_classes=['vehicle'])
        else:
            raise ValueError("Unexpected `_type`(='{0}')".format(_type))
        
    def get_ego_waypoint_list(self, **kwargs):
        """
        The ego is on road segment.
        """
        longitudinal_resolution = self.config.longitudinal_resolution
        route = [[0, -1, 'start'], [128, -1, 'start']]
        
        wp_list = route_utils.sample_waypoints_along_route(self.map_object, route, 0.0, longitudinal_resolution)

        return wp_list
    
    def get_occluder_waypoint_group_list(self, ego):
        """
        The occluders are virtual static
        """
        # first occluder
        road_id,lane_id = 37,2
        s = 35.0
        road = self.map_object.getRoad(road_id)
        wp1 = road.calcWaypoint(0, lane_id, s)

        # second occluder
        lane_id = -2
        wp2 = road.calcWaypoint(0, lane_id, s)

        wp_group_list = [
            [wp1, wp2]
        ]

        return wp_group_list
    
    def get_occludee_waypoint_list(self, ego, occluder_group, **kwargs):
        """
        There are multiple occluders.
        """
        longitudinal_resolution = self.config.longitudinal_resolution
        route = [[37, -1, 'start'], [145, -1, 'start']]
        init_s = 55.0

        wp_list = route_utils.sample_waypoints_along_route(self.map_object, route, init_s, longitudinal_resolution)

        return wp_list
    
    def sample_actor_group_sequentially(self, actor_group_sampler, object_start_no=None):
        assert object_start_no is None or object_start_no>=0

        type_id, waypoint_group = actor_group_sampler.get_sample_sequentially()
        if type_id is None:
            return None
        
        # get actor
        if object_start_no is not None:
            self.oim.reset_to(object_start_no)
        
        # assume actors in a group share the same type id
        actor_group = [
            trasc.get_actor_from_actor_type(self.atm, type_id, self.oim, waypoint) 
            for waypoint in waypoint_group
        ]
        return actor_group
    
    def sample_ego(self, sampler):
        return self.sample_actor_sequentially(sampler, 0)
    
    def sample_occluder_group(self, sampler):
        """Sample multiple occluders at one time.
        """
        return self.sample_actor_group_sequentially(sampler, 1)
    
    def sample_occludee(self, sampler):
        # there have been one ego and two occluders
        return self.sample_actor_sequentially(sampler, 3)
    
    def sample_others_ar(self, ego, occluder_group, occludee, seed, eval_range):
        """Sample other road users in auto regressive manner.
        TODO get route automatically.
         
        """
        self.oim.reset_to(2+len(occluder_group))
        random.seed(seed)
        self.vehicle_sampler.set_seed(seed)

        all_other_vehicle_list = []
        minimal_dist_to_leading_vehicle = 10.0  # front bumper to rear bumper

        # 1) Sample ego's leading and following
        route = [[0, -1, 'start'], [128, -1, 'start'], [1, -1, 'start']]
        # 1.1) lv: in front the exit
        ego_front_route_s = route_utils.calc_route_s_for_actor(self.map_object, route, ego, 'front')
        # exit in road segment center
        exit_route_s = self.map_object.getRoad(0).length + self.map_object.getRoad(128).length*0.5
        init_s = max(ego_front_route_s+minimal_dist_to_leading_vehicle, exit_route_s)
        vehicle_list = scene_utils.sample_vehicles_along_route(self.map_object, init_s, route, self.vehicle_sampler)
        all_other_vehicle_list.extend(vehicle_list)

        # 1.2) fv
        ego_rear_route_s = route_utils.calc_route_s_for_actor(self.map_object, route, ego, 'rear')
        if ego_rear_route_s > 0:
            vehicle_list = scene_utils.sample_vehicles_against_route(self.map_object, ego_rear_route_s, route, self.vehicle_sampler)
            all_other_vehicle_list.extend(vehicle_list)

        # 2) Sample other vehicles on ego's adjacent lane
        route = [[1, 1, 'end'], [127, 1, 'end'], [0, 1, 'end']]
        vehicle_list = scene_utils.sample_vehicles_along_route(self.map_object, 0.0, route, self.vehicle_sampler)
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

    
    def has_occlusion_among_key_actors(self, ego, occluder_group, occludee, two_vertices_group=None):
        """
        Returns True if there is occlusion
        """
        if two_vertices_group is None:
            two_vertices_group = [None for _ in range(len(occluder_group))]
        
        for occluder,two_vertices in zip(occluder_group, two_vertices_group):
            one_flag = super().has_occlusion_among_key_actors(
                ego, occluder, occludee, two_vertices
            )
            if one_flag:
                return True
        return False
    
    def has_collision_among_key_actors(self, ego, occluder_group, occludee):
        """ In most of the geneation process, collision won't happen.
        Rewrite this when collision is possible.

        Returns
        -------
        flag : bool
            Returns True if there is collision among these actors.
        """
        flag = False
        for occluder in occluder_group:
            one_flag = self.has_collision_among_actors([ego, occluder, occludee])
            if one_flag:
                flag = True
                break
        return flag
    
    def check_and_construct_scene(self, ego, occluder_group, occludee, npc_list, perception_range=70.0, occluder_as_cav=True, ignore_none_valid_cav=False):
        """Check there exists at least one car can see occludee.

        Parameters
        ----------
        ego : CarlaActor

        occluder_group : list[CarlaActor]

        occludee : CarlaActor

        npc_list : list

        perception_range : float
            The CAV's perception range.

        occluder_as_cav : bool
            If set, the occluder can also taken as CAV.

        ignore_none_valid_cav : bool
            If set, returns True even if none of CAV can see the occludee 
            clearly. We set it True when existing a RSU.

        Returns
        -------
        flag : bool
            Returns True if we decided to construct the scene after checking.
        """
        # 1) Check visivility
        # occludee must locate in perception range of CAV
        # CAV can be: occluder, other npc
        cav_list = []
        actor_list = occluder_group + npc_list if occluder_as_cav else npc_list
        for actor in actor_list:
            if actor.vclass == 'car' and actor.distance(occludee) <= perception_range:
                cav_list.append(actor)
        
        valid_cav_ids = self.check_can_see_target(
            cav_list,
            [ego, occludee] + occluder_group + npc_list,
            occludee,
            self.mounting_height
        )

        # 2) Save scene if possible
        # valid scenes: there exists valid cav or ignore_none_valid_cav or occludee is of car
        if len(valid_cav_ids)>0 or ignore_none_valid_cav or occludee.vclass == 'car':
            # 2) construct scene
            all_actors = [ego] + occluder_group + [occludee] + npc_list
            related_road_ids = list(set([actor.waypoint.road_id for actor in all_actors]))
            occlusion_pairs = [[occluder.id, occludee.id, valid_cav_ids] for occluder in occluder_group]
            scene = trasc.TrafficScene(
                self.tsim.get_id(),
                self.map_name,
                related_road_ids,
                ego.id,
                copy.deepcopy(all_actors),
                occlusion_pairs
            )
            self.scene_list.append(scene)
            return True
        
        return False
    
    def run(self):
        # 0) Initialization
        eval_range = self.config.eval_range
        npc_sampling_times = self.config.npc_sampling_times

        # 1) Sample ego
        # 1.1) init ego sampler
        ego_sampler = my_sampler.EgoSampler(
            self.get_type_sampler_for("ego"),
            self.get_ego_waypoint_list()
        )
        
        occluder_type_sampler = self.get_type_sampler_for("occluder")
        
        # stats
        type_scene_count = {_type: [0,0] for _type in occluder_type_sampler.get_all_types()}
        num_total_scenes = 0
        num_total_occludee_sampling_times = 0
        num_total_npc_sampling_times = 0
        num_critical_occludee = 0

        ego_count = 0
        start_time = time.time()
        logging.info("Start generating scenes for {0}".format(self.name))
        logging.info("Config: {0}".format(self.config))
        while True:
            # 1.2) sample an ego
            ego = self.sample_ego(ego_sampler)
            if ego is None:
                break
            if ego.id > 0:
                logging.error("unexpected ego id: {0} for {1}-th ego".format(ego.id, ego_count))

            ego_count += 1

            # 2) Sample occluder
            # 2.1) init sampler
            occluder_group_sampler = OccluderGroupSampler(
                occluder_type_sampler,
                self.get_occluder_waypoint_group_list(ego)
            )

            while True:
                # 2.2) Sample occludr group
                occluder_group = self.sample_occluder_group(occluder_group_sampler)
                if occluder_group is None:
                    break

                # precompute to save time
                two_vertices_group = [
                    occluder.bounding_box.get_two_vertices_for_circular_sector(ego.bounding_box.location)
                    for occluder in occluder_group
                ]
                

                # 3) Sample occludee
                # 3.1) Init sampler
                # get occludee's waypoint
                occludee_sampler = my_sampler.OccludeeSampler(
                    self.get_type_sampler_for("occludee"),
                    self.get_occludee_waypoint_list(ego, occluder_group)
                )

                num_cur_scenes = 0
                while True:
                    # 3.2) get occludee
                    occludee = self.sample_occludee(occludee_sampler)
                    num_total_occludee_sampling_times += 1
                    type_scene_count[occluder_group[0].type_id][0] += 1  # assume share the same type_id
                    if occludee is None:
                        break
                    
                    # 3.3) check: ensure occludee is actually occluded and no collision
                    # check occlusion
                    flag = self.has_occlusion_among_key_actors(ego, occluder_group, occludee, two_vertices_group)
                    if not flag:
                        continue

                    
                    # check collision 
                    flag = self.has_collision_among_key_actors(ego, occluder_group, occludee)
                    if flag:
                        continue
                    num_critical_occludee += 1

                    # 4) sample others
                    for i in range(npc_sampling_times):
                        # 4.1) sample in auto-regressive manner
                        num_total_npc_sampling_times += 1
                        seed = num_total_npc_sampling_times
                        all_other_vehicle_list = self.sample_others_ar(ego, occluder_group, occludee, seed, eval_range)

                        # 4.2) check
                        # 4.2.1) other checks: such as collision
                        flag = self.check_scene_misc(ego, occluder_group, occludee, all_other_vehicle_list)
                        if not flag:
                            continue

                        # 4.2.2) check visivility and then construct the scene
                        flag = self.check_and_construct_scene(ego, occluder_group, occludee, all_other_vehicle_list, 
                                                              perception_range=self.config.perception_range, 
                                                              occluder_as_cav=self.config.occluder_as_cav, 
                                                              ignore_none_valid_cav=self.config.ignore_none_valid_cav
                                                              )
                        if flag:
                            type_scene_count[occluder_group[0].type_id][1] += 1
                            num_cur_scenes += 1
                
                # print summary
                num_total_scenes += num_cur_scenes

        # print summary
        end_time = time.time()
        msg = "generation for {0} egos finished, {1}/{2} occludees --> {3} scenes, consuming: {4}s".format(
            ego_count, num_critical_occludee, num_total_occludee_sampling_times, num_total_scenes, end_time-start_time
        )
        logging.info(msg)
        logging.info(type_scene_count)
        
        # save result
        if self.save_path:
            self.save_scenes(self.save_path)

    def unitest(self):
        pass


if __name__ == '__main__':
    # generate scenes
    logging.basicConfig(format='%(levelname)s: %(message)s', level=logging.INFO)

    # replace as your local path
    map_path = "/home/lrs/scripts/CoRiskyScene/coriskyscene/data_collection/data/map_opendrive/Town07.xodr"

    gen = ParkingExitSceneGenerator("parkingexit", map_path)
    gen.run()
