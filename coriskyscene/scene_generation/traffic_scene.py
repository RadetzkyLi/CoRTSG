#!/usr/bin/env python
# -*- coding: utf-8 -*-
'''
@File    :   traffic_scene.py
@Date    :   2024-01-10
@Author  :   Rongsong Li <rongsong.li@qq.com>
@Version :   1.0
@Desc    :   Definitions and operations relevant to a traffic scene
'''

import os
import sys 
import json
import math
from lxml import etree
import numpy as np
from collections import namedtuple

from coriskyscene.opendriveparser.elements import coord
from coriskyscene.occlusion import occlusion_model as occm


# ===============================================================
# - Scene data structure
# ===============================================================
CarlaActorType = namedtuple("CarlaActorType", 
                            ["id", "x", "y", "z", "vclass"])
CarlaActorType.__doc__ = """Carla built in actor type
Args:
    id: {str}, unique type id
    x: {float}, half length
    y: {float}, half width
    z: {float}, half height
    vclass: {str}, actor class, one of {"car", "van", "truck", "authority", 
        "bicycle", "motorcycle", "pedestrian", "static"}
"""

class CarlaActor:
    def __init__(self, id, type_id, vclass, bounding_box, waypoint=None) -> None:
        self.id = id
        self.type_id = type_id
        self.vclass = vclass
        self.bounding_box = bounding_box
        self.waypoint = waypoint

    def __repr__(self) -> str:
        return str(self.to_dict())
        return f"CarlaActor(id={self.id}, type_id={self.type_id}, vclass={self.vclass}, "\
                "bounding_box={self.bounding_box}, waypoint={self.waypoint})"
    
    def to_dict(self) -> dict:
        """Convet info of a actor into dict"""
        out_dict = {
            "id": self.id,
            "type_id": self.type_id,
            "vclass": self.vclass,
            "waypoint": self.waypoint._asdict(),
            "extent": [self.bounding_box.extent.x, self.bounding_box.extent.y, self.bounding_box.height*0.5]
        }
        return out_dict
    
    def set_waypoint(self, new_waypoint):
        x,y = new_waypoint.coord[0],new_waypoint.coord[1]
        yaw = np.rad2deg(new_waypoint.heading)
        self.bounding_box.set_transform(x, y, yaw)
        self.waypoint = new_waypoint
        return self
    
    def distance(self, other: 'CarlaActor'):
        """Calculate distance to other actor.
        
        Paramters
        ---------
        other : CarlaActor
        
        Returns
        -------
        d : float
            distance in meters
        """
        d = self.bounding_box.location.distance(other.bounding_box.location)
        return d

TrafficScene = namedtuple("TrafficScene", 
                          ["id", "town_name", "relevant_roads", 
                           "ego_id", "actor_list", "occlusion_pairs"
                           ])
TrafficScene.__doc__ = """Definition of traffic scene. 
Parameters
----------
id : str
    The scene's identifier.

town_name : str
    The used OpenDRIVE map name

relevant_roads : list[int]
    The relevant OpenDRIVE road ids, junctions are also included.

ego_id : int
    The ego's id

actor_list : list[CarlaActor]
    Info of all actors including static obstacles.

occlusion_pairs : list
    Each element consists of (occluder_id, occludee_id, cav_ids) in 
    which only cav that can see occludee are included

Notes
-----
1. We follow the 3 layer scenario structure proposed by PEGASUS project,
    i.e., functional scenario --> logical scenario --> concrete scenario.
    A traffic scene is snapshot of a concrete scenario.
"""

class ObjectIdManager:
    """Make sure no repetitive id in a scene"""
    def __init__(self) -> None:
        self.count = 0

    def get_id(self):
        """Returns a non-repetitive id for object"""
        _id = self.count
        self.count += 1
        return _id
    
    def reset_to(self, val:int):
        """reset to given val"""
        assert val>=0
        self.count = int(val)

class TrafficSceneIdManager:
    """Make sure a scene has a unique id, i.e., 
    "{logical_scenario_name}_{count}

    Attributes
    ----------
    count : int
        Call times of the function `get_id()`. It's a class variable
        and instances share the same one.

    """
    count = 0
    def __init__(self, logical_scenario_name:str) -> None:
        self.logical_scenario_name = logical_scenario_name

    def get_id(self):
        """Returns scene's id sequentially, e.g., 0,1,2,...
        
        Notes
        -----
        1. Wherever you instancialize and call this method, the count
            will accumulate.
        """
        _id = TrafficSceneIdManager.count
        TrafficSceneIdManager.count += 1
        return "%s_%05d"%(self.logical_scenario_name, _id)

class ActorTypeManager:
    def __init__(self) -> None:
        self._init_types()

    @property
    def types(self):
        return self._types
    
    def _init_types(self):
        """
        
        Notes:
            'vehicle.mercedes.sprinter' is in fact van.
            'vehicle.nissan.patrol_2021' is SUV of height more than 2.0 meters.
            'vehicle.jeep.wrangler_rubicon' is SUV of height 1.9 meters.
            'vehicle.nissan.patrol' is SUV of height 1.9 meters
        """
        self._types = [
            CarlaActorType(id='vehicle.audi.a2', x=1.852684736251831, y=0.8943392634391785, z=0.7745251059532166, vclass='car'),
            CarlaActorType(id='vehicle.mercedes.sprinter', x=2.957595109939575, y=0.9942164421081543, z=1.2803276777267456, vclass='van'),
            CarlaActorType(id='vehicle.chevrolet.impala', x=2.6787397861480713, y=1.0166014432907104, z=0.7053293585777283, vclass='car'),
            CarlaActorType(id='vehicle.citroen.c3', x=1.9938424825668335, y=0.9254241585731506, z=0.8085547685623169, vclass='car'),
            CarlaActorType(id='vehicle.tesla.model3', x=2.3958897590637207, y=1.081725001335144, z=0.744159996509552, vclass='car'),
            CarlaActorType(id='vehicle.dodge.charger_police_2020', x=2.6187572479248047, y=1.0485419034957886, z=0.819191575050354, vclass='authority'),
            CarlaActorType(id='vehicle.micro.microlino', x=1.1036475896835327, y=0.7404598593711853, z=0.6880123615264893, vclass='car'),
            CarlaActorType(id='vehicle.dodge.charger_police', x=2.487122058868408, y=1.0192005634307861, z=0.7710590958595276, vclass='authority'),
            CarlaActorType(id='vehicle.mercedes.coupe_2020', x=2.3368194103240967, y=1.0011461973190308, z=0.7209736704826355, vclass='car'),
            CarlaActorType(id='vehicle.harley-davidson.low_rider', x=1.1778701543807983, y=0.38183942437171936, z=0.6382853388786316, vclass='motorcycle'),
            CarlaActorType(id='vehicle.dodge.charger_2020', x=2.5030298233032227, y=1.0485419034957886, z=0.7673624753952026, vclass='car'),
            CarlaActorType(id='vehicle.ford.ambulance', x=3.18282151222229, y=1.1755871772766113, z=1.215687870979309, vclass='truck'),
            CarlaActorType(id='vehicle.lincoln.mkz_2020', x=2.44619083404541, y=1.115301489830017, z=0.7400735020637512, vclass='car'),
            CarlaActorType(id='vehicle.mini.cooper_s_2021', x=2.2763495445251465, y=1.0485360622406006, z=0.8835831880569458, vclass='car'),
            CarlaActorType(id='vehicle.toyota.prius', x=2.256761312484741, y=1.0034072399139404, z=0.7624167203903198, vclass='car'),
            CarlaActorType(id='vehicle.ford.mustang', x=2.358762502670288, y=0.947413444519043, z=0.650469958782196, vclass='car'),
            CarlaActorType(id='vehicle.volkswagen.t2', x=2.2402184009552, y=1.034657597541809, z=1.0188959836959839, vclass='van'),
            CarlaActorType(id='vehicle.carlamotors.firetruck', x=4.234020709991455, y=1.4455441236495972, z=1.9137061834335327, vclass='truck'),
            CarlaActorType(id='vehicle.carlamotors.carlacola', x=2.601919174194336, y=1.3134948015213013, z=1.2337223291397095, vclass='truck'),
            CarlaActorType(id='vehicle.vespa.zx125', x=0.9023334980010986, y=0.42784383893013, z=0.6178141832351685, vclass='motorcycle'),
            CarlaActorType(id='vehicle.nissan.patrol_2021', x=2.782914400100708, y=1.0749834775924683, z=1.0225735902786255, vclass='car'),
            CarlaActorType(id='vehicle.lincoln.mkz_2017', x=2.4508416652679443, y=1.0641621351242065, z=0.7553732395172119, vclass='car'),
            CarlaActorType(id='vehicle.tesla.cybertruck', x=3.1367764472961426, y=1.1947870254516602, z=1.049095630645752, vclass='truck'),
            CarlaActorType(id='vehicle.audi.etron', x=2.427854299545288, y=1.0163782835006714, z=0.8246796727180481, vclass='car'),
            CarlaActorType(id='vehicle.seat.leon', x=2.0964150428771973, y=0.9080929160118103, z=0.7369155883789062, vclass='car'),
            CarlaActorType(id='vehicle.diamondback.century', x=0.8214218020439148, y=0.18625812232494354, z=0.5979812741279602, vclass='bicycle'),
            CarlaActorType(id='vehicle.gazelle.omafiets', x=0.9177202582359314, y=0.16446444392204285, z=0.5856872797012329, vclass='bicycle'),
            CarlaActorType(id='vehicle.bmw.grandtourer', x=2.3055028915405273, y=1.1208566427230835, z=0.8336379528045654, vclass='car'),
            CarlaActorType(id='vehicle.bh.crossbike', x=0.7436444163322449, y=0.42962872982025146, z=0.5397894978523254, vclass='bicycle'),
            CarlaActorType(id='vehicle.kawasaki.ninja', x=1.0166761875152588, y=0.4012899398803711, z=0.5727267861366272, vclass='motorcycle'),
            CarlaActorType(id='vehicle.yamaha.yzf', x=1.1047229766845703, y=0.43351709842681885, z=0.6255727410316467, vclass='motorcycle'),
            CarlaActorType(id='vehicle.audi.tt', x=2.0906050205230713, y=0.9970585703849792, z=0.6926480531692505, vclass='car'),
            CarlaActorType(id='vehicle.jeep.wrangler_rubicon', x=1.9331103563308716, y=0.9525982737541199, z=0.9389679431915283, vclass='car'),
            CarlaActorType(id='vehicle.nissan.patrol', x=2.3022549152374268, y=0.9657964706420898, z=0.9274230599403381, vclass='car'),
            CarlaActorType(id='vehicle.nissan.micra', x=1.8166879415512085, y=0.9225568771362305, z=0.7506412863731384, vclass='car'),
            CarlaActorType(id='vehicle.mini.cooper_s', x=1.9029000997543335, y=0.985137939453125, z=0.7375151515007019, vclass='car'),
            CarlaActorType(id='vehicle.mercedes.coupe', x=2.5133883953094482, y=1.0757731199264526, z=0.8253258466720581, vclass='car'),
            CarlaActorType(id='walker.pedestrian.0010', x=0.25, y=0.25, z=0.550000011920929, vclass='pedestrian'),
            CarlaActorType(id='walker.pedestrian.0001', x=0.18767888844013214, y=0.18767888844013214, z=0.9300000071525574, vclass='pedestrian'),
            CarlaActorType(id='virtual.static.0000', x=12.500, y=0.1, z=5.000, vclass='static'),
            CarlaActorType(id='virtual.static.0001', x=30.000, y=0.1, z=5.000, vclass='static'),
            CarlaActorType(id='virtual.static.0002', x=35.000, y=0.1, z=5.000, vclass='static'),
            CarlaActorType(id='virtual.static.0003', x=37.000, y=0.1, z=5.000, vclass='static')
        ]

    @staticmethod
    def load_actor_types(path: str):
        """Load actor type definition"""
        actor_type_list = []
    
        with open(path, 'r') as f:
            root = etree.parse(f).getroot()
        for vtype in root.findall("vType"):
            vclass = vtype.get("vClass")
            if vclass == "pedestrian":
                # most of the pedestrians share the same bounding box, so, we just 
                # consider the following two
                if vtype.get("id") not in ["walker.pedestrian.0001", "walker.pedestrian.0010"]:
                    continue
            actor_type_list.append(CarlaActorType(
                vtype.get("id"),
                float(vtype.get("length"))*0.5,
                float(vtype.get("width"))*0.5,
                float(vtype.get("height"))*0.5,
                vclass
            ))
        return actor_type_list
    
    def get_actor_type(self, type_id):
        """ 
        Parameters
        ----------
        type_id : str|int
            If str, it's CARLA type id; if int, it's index of _types.
        Returns
        -------
        actor_type : CarlaActorType
        """ 
        if isinstance(type_id, int):
            return self.types[type_id]
        elif isinstance(type_id, str):
            actor_type = [_type for _type in self.types if _type.id == type_id]
            if len(actor_type) == 0:
                return None
            actor_type = actor_type[0]
        else:
            return None
        return actor_type


# ==========================================================
# - common operations to a traffic scene
# ==========================================================
def save_traffic_scenes(scene_list, save_path):
    """Convert the scene into dict and save"""
    class NdarrayEncoder(json.JSONEncoder):
        def default(self, obj):
            if isinstance(obj, np.ndarray):
                return obj.tolist()
            elif isinstance(obj, (np.integer, np.floating, np.bool_)):
                return obj.item()
            return json.JSONEncoder.default(self, obj)
    
    with open(save_path, 'w', encoding='utf8') as f:
        dict_scenes = {"scenes": []}
        for scene in scene_list:
            actors = [actor.to_dict() for actor in scene.actor_list]
            dict_scene = scene._asdict()
            dict_scene["actor_list"] = actors
            dict_scenes['scenes'].append(
               dict_scene
            )
        _str = json.dumps(dict_scenes, cls=NdarrayEncoder, ensure_ascii=False, indent=4)
        f.write(_str)

def load_traffic_scenes(path, as_dict=False):
    """Load traffic scnes"""
    with open(path, 'r', encoding='utf8') as f:
        data = json.load(f)
    if as_dict:
        return data
    
    scene_list = []
    for dict_scene in data["scenes"]:
        actor_list = []
        for dict_actor in dict_scene["actor_list"]:
            dict_actor["waypoint"]["coord"] = np.array(dict_actor["waypoint"]["coord"])
            wp = coord.Waypoint(**dict_actor["waypoint"])
            # convert to BoundingBox and CarlaActor
            bbox = occm.BoundingBox(
                occm.Vector2D(dict_actor["extent"][0], dict_actor["extent"][1]),
                occm.Location(wp.coord[0], wp.coord[1]),
                occm.Rotation(np.rad2deg(wp.heading)),
                dict_actor["extent"][2]*2.0
            )
            actor = CarlaActor(
                dict_actor["id"],
                dict_actor["type_id"],
                dict_actor["vclass"],
                bbox, 
                wp
            )
            actor_list.append(actor)
        scene = TrafficScene(
            dict_scene["id"],
            town_name=dict_scene["town_name"],
            relevant_roads=dict_scene["relevant_roads"],
            ego_id=dict_scene["ego_id"],
            actor_list=actor_list,
            occlusion_pairs=dict_scene["occlusion_pairs"]
        )
        scene_list.append(scene)
    return scene_list


# ===============================================================
# - utility
# ===============================================================
def get_actor_from_actor_type(atm, type_id, oim, wp=None):
    """Get the actor from actor type and waypoint.
    
    Parameters
    ----------
    atm : ActorTypeManager

    type_id : str
        Actor's type id

    oim : ObjectIdManager

    wp : Waypoint

    Returns
    -------
    actor : CarlaActor
        The resulting actor
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
    actor = CarlaActor(oim.get_id(), type_id, actor_type.vclass, bbox, wp)
    return actor

def get_bbox_from_actor_type(atm, type_id, wp=None):
    """Get the bounding box from actor type
    
    Parameter
    ---------
    atm : ActorTypeManager

    type_id : str
        Actor's type id

    wp : Waypoint

    Returns
    -------
    bbox : BoundingBox
        The resulting bounding box
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

def calc_distance_between_waypoints(wp1, wp2, dist_type="2d"):
    """Calculate the distance between two waypoints.
    
    Parameters
    ----------
    wp1 : Waypoint

    wp2 : Waypoint

    Returns
    ------
    d : float
        in meters
    """
    if dist_type == '2d':
        arr = [wp1.coord[0]-wp2.coord[0], wp1.coord[1]-wp2.coord[1]]
    elif dist_type == '3d':
        arr = [wp1.coord[0]-wp2.coord[0], wp1.coord[1]-wp2.coord[1], wp1.coord[2]-wp2.coord[2]]
    else:
        raise ValueError("Unknown `dist_type`({0})".format(dist_type))
    
    d = math.sqrt(sum([x**2 for x in arr]))
    return d