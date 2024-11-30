#!/usr/bin/env python
# -*- coding: utf-8 -*-
'''
@File    :   example_scene_generator.py
@Date    :   2024-02-28
@Author  :   Rongsong Li <rongsong.li@qq.com>
@Version :   1.0
@Desc    :   Generate scenes containing all cycle and motorcycle
'''

import os
from lxml import etree
import copy

from coriskyscene.opendriveparser import parser as odr_parser
from coriskyscene.scene_generation.generator import base_scene_generator as scgen
from coriskyscene.scene_generation import traffic_scene as trasc

class ExampleSceneGenerator(scgen.BaseSceneGenerator):
    """
    
    """
    def __init__(self, name:str, map_path=None) -> None:
        # load opendrive map
        with open(map_path, 'r') as f:
            map_object = odr_parser.parse_opendrive(etree.parse(f).getroot())
        super().__init__(name, map_object)

        self.map_name = os.path.basename(map_path).split(".xodr")[0]

        # set save dir
        save_dir = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "data")
        self.set_save_dir(save_dir)


    def run(self):
        """

        """
        ego_type_id = 'vehicle.lincoln.mkz_2017'
        road_id,lane_id,s_pos = 5,-1,30
        ego_wp = self.map_object.getRoad(road_id).calcWaypoint(0, lane_id, s_pos)
        ego = trasc.get_actor_from_actor_type(self.atm, ego_type_id, self.oim, ego_wp)

        npc_type_ids = [
            # motor
            'vehicle.harley-davidson.low_rider',
            'vehicle.vespa.zx125',
            'vehicle.kawasaki.ninja',
            'vehicle.yamaha.yzf',
            # cycle
            'vehicle.diamondback.century',
            'vehicle.gazelle.omafiets',
            'vehicle.bh.crossbike'
        ]
        npc_wp_configs = [
            [5, 1, 35],
            [5, -1, 35],
            [5, -2, 35],
            [5, 1, 30],
            [5, -2, 30],
            [5, 1, 25],
            [5, -1, 25],
            [5, -2, 25]
        ]
        
        npc_list = []
        for i,type_id in enumerate(npc_type_ids):
            wp_config = npc_wp_configs[i]
            wp = self.map_object.getRoad(wp_config[0]).calcWaypoint(0, wp_config[1], wp_config[2])
            npc = trasc.get_actor_from_actor_type(self.atm, type_id, self.oim, wp)
            npc_list.append(npc)

        # 2) construct scene
        all_actors = [ego] + npc_list
        related_road_ids = list(set([actor.waypoint.road_id for actor in all_actors]))
        scene = trasc.TrafficScene(
            self.tsim.get_id(),
            self.map_name,
            related_road_ids,
            ego.id,
            copy.deepcopy(all_actors),
            []
        )
        self.scene_list.append(scene)


        # save result
        if self.save_path:
            self.save_scenes(self.save_path)


if __name__ == '__main__':
    # replace as your local path
    map_path = "/home/lrs/scripts/CoRiskyScene/coriskyscene/data_collection/data/map_opendrive/Town05.xodr"
    gen = ExampleSceneGenerator("example", map_path)

    gen.run()