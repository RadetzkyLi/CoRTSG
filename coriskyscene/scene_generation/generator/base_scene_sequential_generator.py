#!/usr/bin/env python
# -*- coding: utf-8 -*-
'''
@File    :   scene_generation.py
@Date    :   2024-01-15
@Author  :   Rongsong Li <rongsong.li@qq.com>
@Version :   1.0
@Desc    :   Sequentially Generate testing scene for V2X perception
'''

import time
import logging

from coriskyscene.occlusion import occlusion_model as occm
from coriskyscene.scene_generation import traffic_scene as trasc
from coriskyscene.scene_generation.samplers import my_sampler
from coriskyscene.scene_generation.generator.base_scene_generator import BaseSceneGenerator


class BaseSceneSequentialGenerator(BaseSceneGenerator):
    """Generate scenes by going through all possibilities.
    
    """
    def __init__(self, name: str, map_object) -> None:
        super().__init__(name, map_object)

    def get_type_sampler_for(self, _type):
        """Get type sampler for ego, occluder and occludee.
        Rewrite this in subclass!!!
        
        Parameters
        ----------
        _type : str
            One of {"ego", "occluder", "occludee"}

        Returns
        -------
        sampler :

        """
        assert _type in ["ego", "occluder", "occludee"], "Unexpected _type '{0}'".format(_type)

    def get_ego_waypoint_list(self, **kwargs):
        pass

    def get_occluder_waypoint_list(self, ego, **kwargs):
        pass

    def get_occludee_waypoint_list(self, ego, occluder, **kwargs):
        pass

    def sample_actor_sequentially(self, actor_sampler, object_start_no=None):
        """Sample actors one by one

        Parameters
        ----------
        actor_sampler : 
            It should support method `get_sample_sequentially()`, which returns
            type_id and waypoint when called.

        object_start_no : int
            If set, the object's id in ObjectIdManager will be reset to 
            `object_start_no`.

        Returns
        -------
        actor : CarlaActor
            Returns None if all actors have been sampled once.
        """
        assert object_start_no is None or object_start_no>=0

        type_id, waypoint = actor_sampler.get_sample_sequentially()
        if type_id is None:
            return None
        
        # get actor
        if object_start_no is not None:
            self.oim.reset_to(object_start_no)
        
        actor = trasc.get_actor_from_actor_type(self.atm, type_id, self.oim, waypoint)
        return actor
    
    def sample_ego(self, sampler):
        return self.sample_actor_sequentially(sampler, 0)
    
    def sample_occluder(self, sampler):
        return self.sample_actor_sequentially(sampler, 1)
    
    def sample_occludee(self, sampler):
        return self.sample_actor_sequentially(sampler, 2)
    
    def sample_others_ar(self, ego, occluder, occludee, seed, eval_range):
        """Rewrite in subclass"""
        pass

    def has_collision_among_key_actors(self, ego, occluder, occludee):
        """ In most of the geneation process, collision won't happen.
        Rewrite this when collision is possible.

        Returns
        -------
        flag : bool
            Returns True if there is collision among these actors.
        """
        return False
    
    def has_occlusion_among_key_actors(self, ego, occluder, occludee, two_vertices=None):
        """For a valuable scene, the view line between ego and occludee
        must be blocked by occluder.
        
        Parameters
        ----------
        ego : CarlaActor

        occluder : CarlaActor

        occludee : CarlaActor

        two_vertices : list
            The two vertices, A and B, of occluder such that, angle between vector OA and OB
            is maximized. O denotes ego. 

        Returns
        -------
        flag : bool
            Returns True if there exists occlusion.
        """
        flag = occm.is_occluded(ego.bounding_box, occluder.bounding_box, 
                                occludee.bounding_box, self.mounting_height, two_vertices)
        return flag

    
    def check_scene_misc(self, ego, occluder, occludee, npc_list, **kwargs):
        """Some other checks before construct the scene.
        Rewrite this function if additional checks are required after
        all vehicles have been sampled.

        Parameters
        ----------
        ego : CarlaActor

        occluder : CarlaActor

        occludee : CarlaActor

        npc_list : list
            Each element is an instance of CarlaActor.
        
        Returns
        -------
        flag : bool
            Returns False if the scene don't pass the checks.
        """
        return True
    
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
        # structure: {type_id: [sampling times, valid number]}
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
            occluder_sampler = my_sampler.OccluderSampler(
                occluder_type_sampler,
                self.get_occluder_waypoint_list(ego)
            )

            while True:
                # 2.2) sample occluder
                occluder = self.sample_occluder(occluder_sampler)
                if occluder is None:
                    break

                # precompute to save time
                two_vertices = occluder.bounding_box.get_two_vertices_for_circular_sector(ego.bounding_box.location)

                # 3) Sample occludee
                # 3.1) init sampler
                # get occludee's waypoint
                occludee_sampler = my_sampler.OccludeeSampler(
                    self.get_type_sampler_for("occludee"),
                    self.get_occludee_waypoint_list(ego, occluder)
                )

                num_cur_scenes = 0
                while True:
                    # 3.2) get occludee
                    occludee = self.sample_occludee(occludee_sampler)
                    num_total_occludee_sampling_times += 1
                    type_scene_count[occluder.type_id][0] += 1
                    if occludee is None:
                        break
                    
                    # 3.3) check: ensure occludee is actually occluded and no collision
                    # 3.3.1) check collision among key actors
                    flag = self.has_collision_among_key_actors(ego, occluder, occludee)
                    if flag:
                        continue
                    
                    # 3.3.2) check occlusion
                    flag = self.has_occlusion_among_key_actors(ego, occluder, occludee, two_vertices)
                    if not flag:
                        continue
                    
                    num_critical_occludee += 1

                    # 4) sample others 
                    for i in range(npc_sampling_times):
                        # 4.1) sample in auto-regressive manner
                        num_total_npc_sampling_times += 1
                        seed = num_total_npc_sampling_times
                        all_other_vehicle_list = self.sample_others_ar(ego, occluder, occludee, seed, eval_range)
                        
                        # 4.2) check
                        # 4.2.1) other checks: such as collision
                        flag = self.check_scene_misc(ego, occluder, occludee, all_other_vehicle_list)
                        if not flag:
                            continue

                        # 4.2.2) check visivility and then construct the scene
                        flag = self.check_and_construct_scene(ego, occluder, occludee, all_other_vehicle_list, 
                                                              perception_range=self.config.perception_range, 
                                                              occluder_as_cav=self.config.occluder_as_cav, 
                                                              ignore_none_valid_cav=self.config.ignore_none_valid_cav
                                                              )
                        if flag:
                            type_scene_count[occluder.type_id][1] += 1
                            num_cur_scenes += 1
                
                # print summary
                num_total_scenes += num_cur_scenes
                msg = "occluder: {0} --> {1} scenes".format(occluder.type_id, num_cur_scenes)
                # logging.debug(msg)

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