#!/usr/bin/env python
# -*- coding: utf-8 -*-
'''
@File    :   scene_generation.py
@Date    :   2024-01-14
@Author  :   Rongsong Li <rongsong.li@qq.com>
@Version :   1.0
@Desc    :   Generate testing scene for V2X perception
'''

import os
import copy
import numpy as np
import itertools
import logging

from coriskyscene.occlusion import occlusion_model as occm
from coriskyscene.scene_generation import traffic_scene as trasc


# =============================================================
# - Generator Config
# =============================================================
COMM_RANGE = 70.0  # meters

class SceneGeneratorConfig:
    """
    config of scene generator

    Attributes
    ----------
    comm_range : float

    eval_range : float

    perception_range : float

    longitudinal_resolution : float

    lateral_resolution : float

    occluder_as_cav : bool
        If set, the occluder can also regarded as CAV.
    
    ignore_none_valid_cav : bool
        If set the scene will be saved even if the none of cav can see
        the target clearly. Set it to True when near RSU.

    npc_sampling_times : int

    """
    def __init__(self,
                 comm_range:float = COMM_RANGE,           # meters
                 eval_range:float = COMM_RANGE,           # meters
                 perception_range:float = COMM_RANGE,     # meters
                 longitudinal_resolution:float = 2.0,     # meters
                 lateral_resolution:float = 1.0,          # meters
                 occluder_as_cav:bool = True,             # 
                 ignore_none_valid_cav:bool = False,      # 
                 npc_sampling_times:int = 1,              # int
                 **kwargs 
                 ) -> None:
        assert comm_range > 0
        assert eval_range > 0
        assert perception_range > 0
        assert longitudinal_resolution > 0
        assert lateral_resolution > 0
        assert npc_sampling_times > 0

        self.comm_range = comm_range
        self.eval_range = eval_range
        self.perception_range = perception_range
        self.longitudinal_resolution = longitudinal_resolution
        self.lateral_resolution = lateral_resolution
        self.occluder_as_cav = occluder_as_cav
        self.ignore_none_valid_cav = ignore_none_valid_cav
        self.npc_sampling_times = npc_sampling_times

        self.kwargs = kwargs

    def __repr__(self) -> str:
        _str = f"SceneGeneratorConfig(comm_range={self.comm_range}, eval_range={self.eval_range}, perception_range={self.perception_range}, "\
                f"longitudinal_resolution={self.longitudinal_resolution}, lateral_resolution={self.lateral_resolution}, "\
                f"occluder_as_cav={self.occluder_as_cav}, ignore_none_valid_cav={self.ignore_none_valid_cav}, "\
                f"npc_sampling_times={self.npc_sampling_times}, kwargs={self.kwargs})"
        return _str

# =============================================================
# - Base Scene Generator
# =============================================================
class BaseSceneGenerator:
    """The base class for scene generator.

    Attributes
    ----------
    map_object : 
        The OpenDRIVE map (of version 1.4)
    
    mounting_height : float
        Sensor's mounting height from the top of a car
    """
    mounting_height = 0.3 # meters
    
    def __init__(self, name:str, map_object) -> None:
        self.map_object = map_object
        self.map_name = None  # initialize in subclass
        self.name = name

        # actor type manager and id manager
        self.atm = trasc.ActorTypeManager()
        self.oim = trasc.ObjectIdManager()
        self.tsim = trasc.TrafficSceneIdManager(name)

        self.scene_list = [] # total scenes

        # config
        self.config = SceneGeneratorConfig()
        self.save_path = None

    @staticmethod
    def get_bbox_from_actor_type(atm, type_id, wp=None):
        """Get the bounding box from actor type
        Parameters
        ----------
        atm :
            Instance of ActorTypeManager

        type_id : str
            The actor's type id.

        wp : Waypoint

        Returns
        -------
        bbox : 

        """
        actor_type = atm.get_actor_type(type_id)
        if wp is None:
            x,y,yaw = 0,0,0
        else:
            x,y = wp.coord[0],wp.coord[1]
            yaw = np.rad2deg(wp.heading)
        bbox = occm.BoundingBox(
            extent=occm.Vector2D(actor_type.x, actor_type.y),
            location=occm.Location(x, y),
            rotation=occm.Rotation(yaw),
            height=actor_type.z*2.0
        )
        return bbox
    
    @staticmethod
    def get_actor_from_actor_type(atm, type_id, oim, wp=None):
        """Get the actor from actor type and waypoint
        
        Parameters
        ----------
        atm : 
            Instance of ActorTyepManager

        type_id : str
            The actor's type id

        oim :
            Instance of ObjectIdManager

        wp : Waypoint 
            The waypoint to spawn the actor

        Returns
        -------
        actor : 
        """
        actor_type = atm.get_actor_type(type_id)
        if wp is None:
            x,y,yaw = 0,0,0
        else:
            x,y = wp.coord[0],wp.coord[1]
            yaw = np.rad2deg(wp.heading)
        bbox = occm.BoundingBox(
            extent=occm.Vector2D(actor_type.x, actor_type.y),
            location=occm.Location(x, y),
            rotation=occm.Rotation(yaw),
            height=actor_type.z*2.0
        )
        actor = trasc.CarlaActor(oim.get_id(), type_id, actor_type.vclass, bbox, wp)
        return actor

    @staticmethod
    def check_can_see_target(actor_list, obstacle_list, target_actor, mounting_height):
        """Judge whether the view line between an actor and the target_acor 
        is occluded by obstacles.
        
        Parameters
        ----------
        actor_list : list
            Only actor of vclass 'car' is regarded as CAV

        obstacle_list : list
            The static obstacles should also be included as actor.

        target_actor : 

        Returns
        -------
        valid_actor_ids : list
            Ids of actors that can see the target clearly.
        """
        valid_actor_ids = []
        for actor in actor_list:
            if not (actor.vclass == 'car'):
                continue
            flag = False  # True denotes occluded
            for obstacle in obstacle_list:
                if actor.bounding_box.location.__eq__(obstacle.bounding_box.location):
                    continue
                flag = occm.is_occluded(actor.bounding_box, obstacle.bounding_box, 
                                target_actor.bounding_box, mounting_height)
                if flag:
                    break
            if not flag:
                valid_actor_ids.append(actor.id)
        return valid_actor_ids
    
    @staticmethod
    def has_collision_among_actors(actor_list):
        """Judge whether there exists collisions between any two actors in actor_list.
        
        Parameters
        ----------
        actor_list : list
            All vehicles, pedestrians, static obstacles are included.

        Returns
        -------
        has_collision : bool
            Returns True if any two actors collides.
        """
        n = len(actor_list)
        if n < 2:
            return False
        
        has_collision = False
        for actor,other in itertools.combinations(actor_list, 2):
            has_collision = actor.bounding_box.intersects(other.bounding_box)
            if has_collision:
                break
        return has_collision
    
    @staticmethod
    def check_actor_id_uniqueness(all_actors):
        """Check whether there exists repetitive id for actors."""
        ids = [actor.id for actor in all_actors]
        if len(ids) == len(set(ids)):
            return True
        return False
    
    @staticmethod
    def filter_actors_by_range(actor_list, center_actor, r:float):
        """Filter out actors out of certain range of the center actor.
        
        Parameters
        ----------
        actor_list : list
            Each actor is a dict: {"type_id": xxx, "waypoint": wp}, or a CarlaActor.
        
        center_actor : dict|CarlaActor

        r : float
            The filter range in meters

        Returns
        -------
        actor_list : list
            The filtered actor list.
        """
        assert r>0.02

        if isinstance(center_actor, dict):
            actor_list = [actor for actor in actor_list if 
                          trasc.calc_distance_between_waypoints(
                              center_actor["waypoint"], actor["waypoint"]
                          ) <=r ]
        else:
            actor_list = [actor for actor in actor_list if
                          trasc.calc_distance_between_waypoints(
                              center_actor.waypoint, actor.waypoint
                          ) <=r ]

        return actor_list
    
    def check_and_construct_scene(self, ego, occluder, occludee, npc_list, perception_range=70.0, occluder_as_cav=True, ignore_none_valid_cav=False):
        """Check there exists at least one car can see occludee.

        Parameters
        ----------
        ego : CarlaActor

        occluder : CarlaActor

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
        # 1) check visivility
        # occludee must locate in perception range of CAV
        # CAV can be: occluder, other npc
        cav_list = []
        actor_list = [occluder] + npc_list if occluder_as_cav else npc_list
        for actor in actor_list:
            if actor.vclass == 'car' and actor.distance(occludee) <= perception_range:
                cav_list.append(actor)
        
        valid_cav_ids = self.check_can_see_target(
            cav_list,
            [ego, occluder, occludee] + npc_list,
            occludee,
            self.mounting_height
        )
        # none of cav can see occludee
        if len(valid_cav_ids) == 0 and not ignore_none_valid_cav:
            return False
        
        # 2) construct scene
        all_actors = [ego, occluder, occludee] + npc_list
        related_road_ids = list(set([actor.waypoint.road_id for actor in all_actors]))
        scene = trasc.TrafficScene(
            self.tsim.get_id(),
            self.map_name,
            related_road_ids,
            ego.id,
            copy.deepcopy(all_actors),
            [[occluder.id, occludee.id, valid_cav_ids]]
        )
        self.scene_list.append(scene)
        return True
    
    def check_and_construct_scene_v2(self, ego, occluder, occludee, npc_list, perception_range=70.0, occluder_as_cav=True, ignore_none_valid_cav=False):
        """Check there exists at least one car can see occludee.
        
        For some scenes, it's common that no npc can see target and 
        communicate with ego concurrently. However, if the occludee is 
        another CAV, it can share info with ego. So, such scene will be 
        kept anyway.

        Parameters
        ----------
        ego : CarlaActor

        occluder : CarlaActor

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
        # CAV can be: occluder, other npc, and even occludee (?)
        cav_list = []
        actor_list = [occluder] + npc_list if occluder_as_cav else npc_list
        for actor in actor_list:
            if actor.vclass == 'car' and actor.distance(occludee) <= perception_range:
                cav_list.append(actor)
        
        valid_cav_ids = self.check_can_see_target(
            cav_list,
            [ego, occluder, occludee] + npc_list,
            occludee,
            self.mounting_height
        )

        # 2) Save valid scene: non empty valid_cav_ids, 
        #    or ignore_none_valid_cav is True, or occludee is of 'car'.
        if len(valid_cav_ids)>0 or ignore_none_valid_cav or occludee.vclass == 'car':
            # 2) construct scene
            all_actors = [ego, occluder, occludee] + npc_list
            related_road_ids = list(set([actor.waypoint.road_id for actor in all_actors]))
            scene = trasc.TrafficScene(
                self.tsim.get_id(),
                self.map_name,
                related_road_ids,
                ego.id,
                copy.deepcopy(all_actors),
                [[occluder.id, occludee.id, valid_cav_ids]]
            )
            self.scene_list.append(scene)
            return True
        
        return False

    def save_scenes(self, save_path):
        """Save generated scenes as JSON to given path"""
        if len(self.scene_list)>0:
            trasc.save_traffic_scenes(self.scene_list, save_path)
        else:
            logging.warning("Give up saving empty scene!")
    
    def set_save_dir(self, _dir:str=""):
        """The result will be save to "{_dir}/{self.name}.json" """
        path = os.path.join(_dir, self.name+".json")
        self.save_path = path

    def sample_ego(self):
        """Sample ego's type and waypoint"""
        pass

    def sample_occluder(self):
        """Sample occluder's type and waypoint"""
        pass

    def sample_occludee(self):
        """Sample occludee's type and waypoint"""
        pass

    def sample_others_ar(self):
        """Sample other's type (vehicles/pedestrians) and waypoint in auto regressive manner"""
        pass

    def run(self):
        """generate scenes for specific logical scenario"""
        pass