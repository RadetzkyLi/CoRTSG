# -*- coding: utf-8 -*-
# Modified by: Rongsong Li <rongsong.li@qq.com>

import numpy as np
import itertools

from coriskyscene.opendriveparser.elements.roadPlanView import PlanView
from coriskyscene.opendriveparser.elements.roadLink import Link
from coriskyscene.opendriveparser.elements.roadLanes import Lanes
from coriskyscene.opendriveparser.elements.roadElevationProfile import (
    ElevationProfile,
)
from coriskyscene.opendriveparser.elements.roadLateralProfile import LateralProfile
from coriskyscene.opendriveparser.elements.junction import Junction
from coriskyscene.opendriveparser.elements.coord import Waypoint
from coriskyscene.opendriveparser.elements.roadObjects import RoadObjects

class Road:
    """ """

    def __init__(self):
        self._id = None
        self._name = None
        # Instance/reference of the junction to which the road belongs as a connecting road
        self._junction = None  
        self._length = None

        self._header = None  # TODO
        self._link = Link()
        self._types = []
        self._planView = PlanView()
        self._elevationProfile = ElevationProfile()
        self._lateralProfile = LateralProfile()
        self._lanes = Lanes()

        self._objects = RoadObjects()

    def __eq__(self, other):
        return self.__dict__ == other.__dict__

    @property
    def id(self):
        """ """
        return self._id

    @id.setter
    def id(self, value):
        """

        Args:
          value:

        Returns:

        """
        self._id = int(value)

    @property
    def name(self):
        """ """
        return self._name

    @name.setter
    def name(self, value):
        """

        Args:
          value:

        Returns:

        """
        self._name = str(value)

    @property
    def length(self):
        """ """
        return self._length
    
    @length.setter
    def length(self, value: float):
        self._length = value

    @property
    def junction(self):
        """ """
        return self._junction

    @junction.setter
    def junction(self, value):
        """

        Args:
          value:

        Returns:

        """
        if not isinstance(value, Junction) and value is not None:
            raise TypeError("Property must be a Junction or NoneType")

        if value == -1:
            value = None

        self._junction = value

    @property
    def link(self):
        """ """
        return self._link

    @property
    def types(self):
        """ """
        return self._types

    @property
    def planView(self):
        """ """
        return self._planView

    @property
    def elevationProfile(self):
        """ """
        return self._elevationProfile

    @property
    def lateralProfile(self):
        """ """
        return self._lateralProfile

    @property
    def lanes(self):
        """ """
        return self._lanes
    
    @property
    def objects(self):
        """ """
        return self._objects
    
    def findLaneSectionByS(self, s_pos: float):
        """Find the lane section correspondingt to given `s_pos`.
        
        Args:
            s_pos: the OpenDRIVE s value.

        Returns:
            lane_section: 
        """
        if s_pos<0 or s_pos>self.length:
            return None
        
        accumulated_s = 0.0
        for lane_section in self.lanes.lane_sections:
            accumulated_s += lane_section.length
            if s_pos<accumulated_s:
                break
        return lane_section
    
    @staticmethod
    def calcDxdyFromW(w: float, h: float, lane_offset: float, is_left: bool=True):
        """Calculate the (dx,dy) for a given width.
        
        Args:
            w: the distance in meters from the center lane. Obtained by lane width.
            h: the heading angle in radians of the road at that point
            lane_offset: the offset in meters of center lane relative to reference 
                line. Positive value indicates an offset to the left.
            is_left: if set, we calculate for left lane.

        Returns:
            [dy, dy]: the offset in x and y direction 
        """
        assert w>=0, "w here cannot be negative!"

        w = w if is_left else -w
        w += lane_offset

        dx = -w * np.sin(h)
        dy = w * np.cos(h)
        return np.array([dx, dy])
    
    @staticmethod
    def calcDxdyFromSt(s: float, t: float, h: float):
        """ Calculate the (dx, dy) relative to origin of s/t coordinate system.
        
        Args:
            s: s-coordinate under s/t frame
            t: t-coordinate under s/t frame
            h: heading angle in radians of s/t frame relative to x/y frame

        Returns:
            [dx,dy]
        """
        dx = s*np.cos(h) - t*np.sin(h)
        dy = s*np.sin(h) + t*np.cos(h)
        return np.array([dx, dy])
    
    def calcWaypoint(self, section_id: int, lane_id: int, s_pos: float):
        """Calculate the Waypoint at the center of specified lane.
        
        Args:
            section_id: OpenDRIVE section's id, based on the order 
                that they are originally defined
            lane_id: OpenDRIVE lane's id, can be positive or negative
            s_pos:  OpenDRIVE s value of the current position

        Returns:
            wp: a Waypoint at the current position
        """
        assert s_pos>=0.0 and s_pos<=self.length, "`s_pos` should lie in [0, {0}], "\
            "got {1}".format(self.length, s_pos)
        
        # retrive the target lane section and lane
        lane_section = self.lanes.getLaneSection(section_id)
        if lane_section is None:
            raise ValueError("Found no lane_section {0} in road {1}"
                             .format(section_id, self.id))
        lane = lane_section.getLane(lane_id)
        if lane is None:
            raise ValueError("Found no lane {0} in road {1}".format(lane_id, self.id))
        
        # x/y coordinates at s_pos on reference line
        xy_pos, tangent = self.planView.calc(s_pos)
        
        cur_lane_offset = self.lanes.calcLaneOffset(s_pos)
        cur_road_elevation = self.elevationProfile.calc(s_pos)

        # calculate t for current lane center
        t = 0.0
        # center lane is zero width
        if lane_id == 0:
            pass
        else:
            step = -1 if lane_id<0 else 1
            for prev_lane_id in range(0, lane_id, step):
                t += lane_section.getLane(prev_lane_id).calcWidth(s_pos)
            target_lane_width = lane_section.getLane(lane_id).calcWidth(s_pos)
            t += 0.5 * target_lane_width
            t = t if lane_id>=0 else -t
        t += cur_lane_offset
        
        dxdy = self.calcDxdyFromSt(0, t, tangent)
        heading = tangent if lane_id<0 else tangent+np.pi

        wp = Waypoint(
            np.append(xy_pos+dxdy, cur_road_elevation),
            heading,
            lane_section.getLane(lane_id).type,
            self.id,
            section_id,
            lane_id,
            s_pos,
            target_lane_width
        )
        return wp


    def calcWaypointInLaneCenter(self, s_pos: float, target_lane_types: list=None):
        """Sample waypoints at all lanes' center using the given s_pos.

        Args:
            s: {float}, the distance in meters along the s-axis from the entry of the road.
            target_lane_types: {list[str]}, by default, the five types are considered:
                "driving", "biking", "sidewalk", "parking", "bidirectional"
        
        Returns:
            wp_list_center: {List}, waypoints along lane center
            wp_list_border: {List}, waypoints along lane border

        Notes:
            1. Currently, superelevation is not considered, i.e., the road surface is
            supposed to parallel with the ground.
            2. The heading at s_pos takes that of reference line, without considering laneOffset.
        """
        assert s_pos>=0.0 and s_pos<=self.length, "`s_pos` should lie in [0, {0}], got {1}".format(
            self.length, s_pos
        )
        if target_lane_types is None:
            target_lane_types = ['driving', 'biking', 'sidewalk', 'parking', 'bidirectional']

        wp_list_center = []  # waypoints along lane center
        wp_list_border = []  # waypoints along lane boundary

        # x/y coordinates at s_pos on reference line
        xy_pos, tangent = self.planView.calc(s_pos)
        
        cur_lane_offset = self.lanes.calcLaneOffset(s_pos)
        cur_road_elevation = self.elevationProfile.calc(s_pos)

        # find corresponding lane section
        lane_section = self.findLaneSectionByS(s_pos)

        # center line
        dxdy = self.calcDxdyFromW(0.0, tangent, cur_lane_offset)
        wp_list_border.append(Waypoint(
            np.append(xy_pos+dxdy, cur_road_elevation),
            tangent,
            "centerLine",
            self.id,
            lane_section.id,
            0, 
            s_pos,
            0.0  
        ))
        
        # go through left and right lanes
        for side_lanes,is_left,hdg in zip([lane_section.leftLanes, lane_section.rightLanes],
                                            [True, False],
                                            [tangent+np.pi, tangent]):
            accumulated_width = 0.0
            for lane in side_lanes:
                width = lane.calcWidth(s_pos)
                
                # lane border
                dxdy = self.calcDxdyFromW(accumulated_width+width, tangent, cur_lane_offset, is_left=is_left)
                wp_list_border.append(Waypoint(
                    np.append(xy_pos+dxdy, cur_road_elevation),
                    hdg,
                    "laneBorder",
                    self.id,
                    lane_section.id,
                    lane.id,
                    s_pos,
                    width
                ))

                # only lanes of target types are considered
                if lane.type in target_lane_types:
                    # lane center
                    dxdy = self.calcDxdyFromW(accumulated_width+width/2.0, tangent, cur_lane_offset, is_left=is_left)
                    wp_list_center.append(Waypoint(
                        np.append(xy_pos+dxdy, cur_road_elevation),  # [x, y, z]
                        hdg,
                        lane.type,
                        self.id,
                        lane_section.id,
                        lane.id,
                        s_pos,
                        width
                    ))

                accumulated_width += width

        return wp_list_center,wp_list_border
    
    def getLaneBorder(self, s_resolution: float=0.5):
        """Obtain the waypoints consistituting lane borders of the road.
        
        Args:
            s_resolution: {float}, the sampling interval in meters along s-axis
            
        Returns:
            wp_border_list: {list}
        """
        if s_resolution>self.length:
            return self.calcWaypointInLaneCenter(0)[1]
        
        wp_border_list = [self.calcWaypointInLaneCenter(s)[1] for s in np.arange(0, self.length, s_resolution)]
        wp_border_list = list(itertools.chain(*wp_border_list))
        return wp_border_list
    
    def calcWaypointInSublaneCenter(self, s_pos: float, lateral_resolution: float, target_lane_types:list=None):
        """Sample waypoints at all sublanes' center using given s_pos. 
        The results can be used by two-wheel vehicles and pedestrians.

        Args:
            s_pos: {float}, the distance in meters along s-axis from the entry of the road
            lateral_resolution: {float}, the sublane width, it is supposed to divide lane 
                width evenly. E.g., if the lane width is 4.0m, the sublane width can be 1m, 0.5m.
        
        Returns:
            wp_list: {List}
        """
        assert s_pos>=0.0 and s_pos<=self.length, "`s_pos` should lie in [0, {0}], got {1}".format(
            self.length, s_pos
        )
        assert lateral_resolution>0.0
        
        if target_lane_types is None:
            target_lane_types = ['driving', 'biking', 'sidewalk', 'parking', 'bidirectional']

        wp_list = []

        # x/y coordinate at s_pos on reference line
        xy_pos, tangent = self.planView.calc(s_pos)

        cur_lane_offset = self.lanes.calcLaneOffset(s_pos)
        cur_road_elevation = self.elevationProfile.calc(s_pos)

        # find target lane section and go through left and right lanes
        lane_section = self.findLaneSectionByS(s_pos)
        for side_lanes,is_left,hdg in zip([lane_section.leftLanes, lane_section.rightLanes],
                                            [True, False],
                                            [tangent+np.pi, tangent]):
            accumulated_width = 0.0
            for lane in side_lanes:
                width = lane.calcWidth(s_pos)
                num_sublanes = int(np.floor(width/lateral_resolution))

                # only consider lane that is of target lane type and with enough width
                if lane.type not in target_lane_types or num_sublanes<2:
                    accumulated_width += width
                    continue

                # go through all possible sublanes
                for i in range(num_sublanes):
                    w = accumulated_width + lateral_resolution*(i+0.5)
                    dxdy = self.calcDxdyFromW(w, tangent, cur_lane_offset, is_left=is_left)
                    wp_list.append(Waypoint(
                        np.append(xy_pos+dxdy, cur_road_elevation), 
                        hdg,
                        lane.type,
                        self.id,
                        lane_section.id,
                        lane.id,
                        s_pos, 
                        width
                    ))
                
                accumulated_width += width

        return wp_list
    
    def calcWaypointInCrosswalk(self, 
                                   longitudinal_resolution: float=0.4, 
                                   lateral_resolution: float=0.4, 
                                   only_border: bool=False):
        """Calculate waypoints in crosswalk area.
        
        Args:
            longitudinal_resolution: {float}, meters, along s-axis
            lateral_resolution: {float}, meters, along t-axis
            only_border: {bool}, if set, only calculate waypoints for border

        Returns:
            wp_list: {list}

        Notes:
            1. Here, we assume the crosswalk is a rectangle centered at (origin_s, origin_t), 
            and prependicular to s-direction of the road.
        """
        wp_list = []

        # go through all road objects
        for road_object in self.objects.objects:
            if not (road_object.type == "crosswalk"):
                continue

            half_width = road_object.width * 0.5
            half_length = road_object.length * 0.5
            origin_s = road_object.origin_s_pos
            origin_t = road_object.origin_t_pos
            z_offset = road_object.z_offset

            # x/y coordinate at s_pos on reference line
            xy_pos, tangent = self.planView.calc(origin_s)

            cur_lane_offset = self.lanes.calcLaneOffset(origin_s)
            cur_road_elevation = self.elevationProfile.calc(origin_s)

            # construct grid 
            if only_border:
                ds_list = [-half_width, half_width]
                # dt_list = [origin_t-half_length, origin_t+half_length]
            else:
                ds_list = np.arange(-half_width, half_width+0.01, longitudinal_resolution)
            dt_list = np.arange(origin_t-half_length, origin_t+half_length+0.01, lateral_resolution)

            # calc for grid point
            heading = tangent + 0.5*np.pi
            z = cur_road_elevation + z_offset
            wp_list.extend([Waypoint(
                np.append(xy_pos+self.calcDxdyFromSt(ds, dt, tangent), z),
                heading,
                "crosswalk",
                self.id,
                0, # only one <objects> element in a road
                road_object.id,
                origin_s+ds,
                road_object.width
            ) for ds,dt in itertools.product(ds_list, dt_list)])
            
        return wp_list