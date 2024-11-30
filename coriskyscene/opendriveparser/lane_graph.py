#!/usr/bin/env python
# -*- coding: utf-8 -*-
'''
@File    :   lane_graph.py
@Date    :   2024-01-15
@Author  :   Rongsong Li <rongsong.li@qq.com>
@Version :   1.0
@Desc    :   Operations to connect lanes
'''

import os
import sys
import numpy as np


class LaneGraph:
    """Find connections among lanes.
    
    Refs
    ----
    1. https://github.com/AlejandroDiazD/opendrive-mapping-planning/blob/master/planning_layer/t4ac_global_planner_ros/src/lane_graph_planner.py
    """
    def __init__(self, map_object) -> None:
        self.map_object = map_object

    def calculate_lane_link(self, road, incomingLane):
        """Calculate the lane link when there is more than on lane section.
        It depends if the lane if forward or backward.

        Parameters
        ----------
        road : 
            
        incomingLane :
        
        Returns
        -------
        outgoingLane_id : int
        """
        # 1) Get incomingEntryLane
        # 1.1) incomingLane direction is forward
        if incomingLane.id < 0:
            # get entryLane
            incomingEntryLane = incomingLane
        # 1.2) incomingLnae direction is backward
        elif incomingLane.id > 0:
            # Get entryLane
            n = len(road.lanes.lane_sections)
            # 1.2.1) Case of 1 single laneSection
            if n == 1:
                incomingEntryLane = incomingLane
            # 1.2.2) Case of multiple lanseSections
            elif n > 1:
                incomingEntryLane = incomingLane
                for i in range(n-1):
                    next_lane_id = incomingEntryLane.link.successorId
                    next_lane = road.lanes.getLaneSection(i+1).getLane(next_lane_id)
                    incomingEntryLane = next_lane

        # 2) Get incomingExitLane
        # 2.1) incomingLane direction if forward
        if incomingLane.id < 0:
            n = len(road.lanes.lane_sections)
            # 2.1.1) Case of 1 single laneSection
            if n == 1:
                incomingExitLane = incomingLane
            elif n > 1:
                incomingExitLane = incomingLane
                for i in range(n-1):
                    next_lane_id = incomingEntryLane.link.successorId
                    next_lane = road.lanes.getLaneSection(i+1).getLane(next_lane_id)
                    incomingExitLane = next_lane
        # 2.2) incomingLane direction is backward
        elif incomingLane.id > 0:
            incomingExitLane = incomingLane

        # 3) Get outgoingEntryLane
        # 3.1) incomingLane direction is forward
        if incomingLane.id < 0:
            outgoingEntryLane_id = incomingExitLane.link.successorId
        # 3.2) incomingLane direction is backward
        elif incomingLane.id > 0:
            outgoingEntryLane_id = incomingExitLane.link.predecessorId

        return incomingEntryLane.id, outgoingEntryLane_id
    

    def calculate_next_connections_for_lane(self, road_id, lane_id):
        """Calculate the next connections for the given lane.
        TODO add support for multiple laneSections
        
        Parameters
        ----------
        road_id : int
            OpenDRIVE's road id.

        lane_id : int
            OpenDRIVE's lane id

        Returns
        -------
        connections : list
            Each connection is like [next_road.id, outgoingEntryLane.id, contactPoint]
        
        Notes
        -----
        1. Only consider unidirection road
        """
        road = self.map_object.getRoad(road_id)
        assert len(road.lanes.lane_sections)==1, "only support 1 single laneSection!"

        lane_conn_list = []
        lane = road.lanes.lane_sections.getLaneSection(0).getLane(lane_id)

        # Take successor as next connection for negative lane ids
        if lane_id < 0 and lane.type == 'driving':
            # When successor is road, get successor connection
            if road.link.successor.elementType == 'road':
                incomingEntryLaneId, outgoingEntryLaneId = \
                    self.calculate_lane_link(road, lane)

                # create connection from current to successor
                lane_conn = [
                    road.link.successor.element_id,
                    outgoingEntryLaneId,
                    road.link.successor.contactPoint
                ]
                lane_conn_list.append(lane_conn)

            # When successor is junction, get all possible connections
            elif road.link.successor.elementType == 'junction':
                junction = self.map_object.getJunction(
                    road.link.successor.element_id
                )
                # create edge from current to all possible connections
                for connection in junction.connections:
                    if connection.incomingRoad == road_id:
                        for laneLink in connection.laneLinks:
                            if laneLink.fromId == lane_id:
                                connectingLaneId = laneLink.toId
                                lane_conn = [
                                    connection.connectingRoad,
                                    connectingLaneId,
                                    connection.contactPoint
                                ]
                                lane_conn_list.append(lane_conn)
        
        # Take predecessor as next connection for positive lane ids
        elif lane_id > 0 and lane.type == 'driving':
            # When predecessor is road, get predecessor connection
            if road.link.predecessor.elementType == 'road':
                incomingEntryLaneId, outgoingEntryLaneId = \
                    self.calculate_lane_link(road, lane)
                # create connection from current to predecessor
                lane_conn = [
                    road.link.predecessor.element_id,
                    outgoingEntryLaneId,
                    road.link.predecessor.contactPoint
                ]

            # When predecessor is junction, get all possible connections
            elif road.link.predecessor.elementType == 'junction':
                junction = self.map_object.getJunction(
                    road.link.predecessor.element_id
                )
                # create connection from current to all possible connections
                for connection in junction.connections:
                    if connection.incomingRoad == road_id:
                        for laneLink in connection.laneLinks:
                            if laneLink.fromId == lane_id:
                                connectingLaneId = laneLink.toId
                                lane_conn = [
                                    connection.connectingRoad,
                                    connectingLaneId,
                                    connection.contactPoint
                                ]
                                lane_conn_list.append(lane_conn)
        return lane_conn_list
    

    def forward_sample(self, road_id, lane_id, s, ds):
        """Get corresponding road and lane id if we go forward with distance `ds`.
        If the next road(s) is in junction, we will go forward again; otherwise

        
        """
        pass