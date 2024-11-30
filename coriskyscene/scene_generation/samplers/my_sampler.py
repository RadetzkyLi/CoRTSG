#!/usr/bin/env python
# -*- coding: utf-8 -*-
'''
@File    :   my_sampler.py
@Date    :   2024-01-16
@Author  :   Rongsong Li <rongsong.li@qq.com>
@Version :   1.0
@Desc    :   Sample actors type, vehicle spacing.
'''

import os
import sys
import pickle
import random
import itertools
import numpy as np
import scipy.stats

# ================================================================
# - Sampler for actor's type
# ================================================================
class BaseActorTypeSamper:
    """Sampler actor's type"""
    def __init__(self) -> None:
        self.sequential_count = 0

        self._types = []  # must be initialzed in subclass

    def __len__(self):
        return len(self._types)   

    def get_all_types(self):
        """Returns all types in a list"""
        return self._types 

    def get_type_sequentially(self):
        """Return all actor's types sequentially.
        
        Returns
        -------
        _type : str
            Returns None if all types have been sampled once.
        """
        if self.sequential_count>=len(self._types):
            return None
        _type = self._types[self.sequential_count]
        self.sequential_count += 1
        return _type

class EgoTypeSampler(BaseActorTypeSamper):
    """Sample ego's type. 
    currently, fix ego as 'vehicle.lincoln.mkz_2017' (same as OPV2V, V2XSet, V2X-ASG)
    
    """
    def __init__(self) -> None:
        super().__init__()

        self._types = ['vehicle.lincoln.mkz_2017']

class OccluderTypeSampler(BaseActorTypeSamper):
    """Sample occluder's type. For CAV, the occluder can be categorized into
    the following 3 classes:
        3 classes                       example when CAV is a car
    1. much lower than CAV              1. small car, e.g., ford.mustang
    2. as nearly high as CAV            2. same type as CAV
    3. much higher than CAV             3. truck, SUV

    
    Parameters
    ----------

    """
    def __init__(self) -> None:
        super().__init__()

        lower_than = [
            'vehicle.ford.mustang', 
            'vehicle.micro.microlino'
        ]
        nearly_high = [
            'vehicle.audi.a2',
            'vehicle.lincoln.mkz_2020',
            # 'vehicle.toyota.prius',
            # 'vehicle.nissan.micra'
        ]
        higher_than = [
            'vehicle.mercedes.sprinter', # van
            # 'vehicle.ford.ambulance',
            # 'vehicle.volkswagen.t2',     # van
            'vehicle.carlamotors.firetruck'
        ]
        self._types = lower_than + nearly_high + higher_than

        self.sequential_count = 0

    def __len__(self):
        return len(self._types)
    
class StaticOccluderTypeSampler(BaseActorTypeSamper):
    """
    For rural curve scene, the occluder is hill/trees which is not
    included in OpenDRIVE, to remedy this, we create virtual 
    bounding boxes to substitute them.

    Parameters
    ----------
    static_types : list
        If None, defaults to ['virtual.static.0002']
    """
    def __init__(self, static_types:list=None) -> None:
        super().__init__()

        if static_types is None:
            self._types = ['virtual.static.0002']
        else:
            self._types = static_types
    
class OccludeeTypeSampler(BaseActorTypeSamper):
    """Sample occludee's type. For CAV, the occludee can be categoried into
    the following 4 classes:
    
    1. much lower than CAV
    2. as nearly high as CAV
    3. much higher than CAV
    4. vulnerable road users (VRU),  e.g., pedestrains, motorcyclist, cyclist

    Parameters
    ----------
    target_main_classes : list[str]
        Any combinations of {"all", "vehicle", "vru", "walker", "nonmotor"}. 
    """
    
    def __init__(self, target_main_classes=['all']) -> None:
        super().__init__()
        supported_main_classes = ['all', 'vehicle', 'vru', 'walker', 'nonmotor']
        assert set(target_main_classes).issubset(supported_main_classes)

        lower_than = [
            'vehicle.ford.mustang', 
            'vehicle.micro.microlino'
        ]
        nearly_high = [
            'vehicle.audi.a2',
            'vehicle.lincoln.mkz_2020',
            # 'vehicle.toyota.prius',
            # 'vehicle.nissan.micra'
        ]
        higher_than = [
            'vehicle.mercedes.sprinter', # van
            # 'vehicle.ford.ambulance',
            # 'vehicle.volkswagen.t2',     # van
            'vehicle.carlamotors.firetruck'
        ]
        VRU = [
            'vehicle.yamaha.yzf',     # motor
            'vehicle.bh.crossbike',   # bike
            'walker.pedestrian.0010', # child
            'walker.pedestrian.0001'  # adult
        ]
        vru_walker = [
            'walker.pedestrian.0010', # child
            'walker.pedestrian.0001'  # adult
        ]
        vru_nonmotor = [
            'vehicle.bh.crossbike',   # bike
            'walker.pedestrian.0010', # child
            'walker.pedestrian.0001'  # adult
        ]
        self._vehicle_types = lower_than + nearly_high + higher_than
        self._vru_types = VRU

        if 'all' in target_main_classes:
            self._types = self._vehicle_types + self._vru_types
        else:
            self._types = []
            if 'vehicle' in target_main_classes:
                self._types.extend(lower_than + nearly_high + higher_than)
            if 'vru' in target_main_classes:
                self._types.extend(VRU)
            if 'walker' in target_main_classes:
                self._types.extend(vru_walker)
            if 'nonmotor' in target_main_classes:
                self._types.extend(vru_nonmotor)
            self._types = list(set(self._types))

        self.sequential_count = 0
        self.sequential_count_vehicle = 0

    def get_all_vehicle_types(self):
        return self._vehicle_types
    
    def get_all_vru_types(self):
        return self._vru_types

    def get_vehicle_type_sequentially(self):
        if self.sequential_count_vehicle >= len(self._vehicle_types):
            return None
        _type = self._vehicle_types[self.sequential_count_vehicle]
        self.sequential_count_vehicle += 1
        return _type

# ==============================================================
# - Distribution
# ==============================================================
class UniformDistribution:
    def __init__(self, v_min:float, v_max:float) -> None:
        assert v_max>v_min
        self.min = v_min
        self.max = v_max
        self.uniform_dist = scipy.stats.uniform()
    
    def rvs(self, size, random_state=None):
        return self.uniform_dist.rvs(loc=self.min, scale=self.max-self.min, 
                                     size=size, random_state=random_state)

# ==============================================================
# - Sampler for actor
# ==============================================================
class BaseActorSampler:
    """Sample actor according to actor's type sampler and actor's
    waypoint sampler, i.e., an actor is determined by its type
    and waypoint. 

    Parameters
    ----------
    type_sampler : 

    waypoint_sampler :

    """
    def __init__(self, type_sampler, waypoint_sampler) -> None:
        self.type_sampler = type_sampler
        self.waypoint_sampler = waypoint_sampler

    @staticmethod
    def reform_actor_types(actor_type_list, target_vclass_list):
        """Convert from list to dict"""
        vclass_to_types = {vclass: [] for vclass in target_vclass_list}
        for actor_type in actor_type_list:
            if actor_type.vclass in target_vclass_list:
                vclass_to_types[actor_type.vclass].append(
                    actor_type
                )
        return vclass_to_types
    

# ==============================================================
# - Sample ego, occluder, occludee and other vehicles
# ==============================================================
class BaseKeyActorSampler(BaseActorSampler):
    """Base sampler for key actors, e.g., ego, occluder, occludee
    
    Parameters
    ----------
    repeat : bool
        If set, the sampling process will be repeated once all samples 
        have been sampled once
    """
    def __init__(self, type_sampler, waypoint_list, waypoint_sampler=None, repeat=False) -> None:
        super().__init__(type_sampler, waypoint_sampler)

        self.repeat = repeat
        self.waypoint_list = waypoint_list
        self._length = int(len(self.type_sampler)) * len(self.waypoint_list)

        self.type_list = self.type_sampler.get_all_types()
        self.sample_generator = itertools.product(self.type_list, self.waypoint_list)

    def refresh_sample_generator(self):
        self.sample_generator = itertools.product(self.type_list, self.waypoint_list)

    def __len__(self):
        return self._length
    
    # def __iter__(self):
    #     return self
    
    # def __next__(self):
    #     return next(self.sample_generator)
    
    def get_sample_sequentially(self):
        """
        
        Returns
        -------
        _type : str
            Ego's type id

        waypoint : Waypoint
        """
        try:
            _type,waypoint = next(self.sample_generator)
        except StopIteration:
            if self.repeat:
                self.refresh_sample_generator()
                _type,waypoint = next(self.sample_generator)
            else:
                _type,waypoint = None,None
        finally:
            return _type,waypoint

class EgoSampler(BaseKeyActorSampler):
    """Sample ego's type and waypoint. 
    
    TODO: add support for continuous waypoints.

    Parameters
    ----------
   
    """
    def __init__(self, type_sampler, waypoint_list, waypoint_sampler=None, repeat=False) -> None:
        super().__init__(type_sampler, waypoint_list, waypoint_sampler, repeat)

class OccluderSampler(BaseKeyActorSampler):
    """Sample occluder's type and waypoint. 
    Currently:
    1. we think waypoints for occluder are discrete and thus limited.
    2. we sample all combinations between types and waypoints sequentially.
    TODO: add support for continuous waypoints.
    
    """
    def __init__(self, type_sampler, waypoint_list, waypoint_sampler=None, repeat=False) -> None:
        super().__init__(type_sampler, waypoint_list, waypoint_sampler, repeat)


class OccludeeSampler(BaseKeyActorSampler):
    """Sample occludee's type and waypoint.
    Currently:
    1. we think waypoints for occludee are discrete and thus limited.
    2. we sample all conbinations between types and waypoints sequentially.
    TODO: add support for continuous waypoints.
    
    """
    def __init__(self, type_sampler, waypoint_list, waypoint_sampler=None, repeat=False) -> None:
        super().__init__(type_sampler, waypoint_list, waypoint_sampler, repeat)


# =========================================================================
# - other vehicle sampler
# =========================================================================
class VehicleSampler(BaseActorSampler):
    """Only sample (other npc) vehicles.
    
    Parameters
    ----------
    atm : ActorTypeManager

    spacing_dist_path : str
        Path of an instance of scipy.stats.rv_continous/scipy.stats.rv_discrete 
        saved in picke file.
    """
    def __init__(self, atm, spacing_dist_path=None) -> None:
        super().__init__(None, None)
        self.atm = atm
        self.spacing_dist = self.load_dist(spacing_dist_path)
        self.target_vclass_list = ['car', 'van', 'truck', 'motorcycle']
        self.vclass_to_types = self.reform_actor_types(
            atm.types, 
            target_vclass_list=self.target_vclass_list
        )
        self.vtypes = [_type for _type in self.atm.types if _type.vclass in self.vclass_to_types]
    
    def load_dist(self, path=None):
        if path is None:
            return UniformDistribution(5, 50)
        with open(path, 'rb') as f:
            dist = pickle.load(f)
        return dist
    
    def set_seed(self, seed):
        self.spacing_dist.random_state = np.random.RandomState(seed)
    
    def random_select_vehicle_type(self):
        vtype = random.choice(self.vtypes)
        return vtype

    def sample_once(self, current_scene=None):
        """Get a sample according to current scene.
        
        TODO: customize sampling process according to current scene. 
        """
        vtype = self.random_select_vehicle_type()
        spacing = self.spacing_dist.rvs(size=1)[0]
        type_id = vtype.id 
        vehicle_half_length = vtype.x
        return type_id,vehicle_half_length,spacing
    
    def sample_leading_vehicle(self, following_vehicle_type_id=None):
        """
        
        TODO: customize sampling process according to following vehicle 
        """
        return self.sample_once()

    def sample_following_vehicle(self, leading_vehicle_type_id=None):
        """
        
        TODO: customize sampling process according to leading vehicle
        """
        return self.sample_once()