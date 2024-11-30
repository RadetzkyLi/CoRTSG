# -*- coding: utf-8 -*-
import numpy as np

from coriskyscene.opendriveparser.elements.road_record import RoadRecord
from coriskyscene.opendriveparser.elements.geometry import Poly3


class Lanes:
    """ """

    def __init__(self):
        self._laneOffsets = []
        self._lane_sections = []

    @property
    def laneOffsets(self):
        """ """
        self._laneOffsets.sort(key=lambda x: x.start_pos)
        return self._laneOffsets

    @property
    def lane_sections(self):
        """ """
        self._lane_sections.sort(key=lambda x: x.sPos)
        return self._lane_sections

    def getLaneSection(self, laneSectionIdx):
        """

        Args:
          laneSectionIdx:

        Returns:

        """
        for laneSection in self.lane_sections:
            if laneSection.idx == laneSectionIdx:
                return laneSection

        return None

    def getLastLaneSectionIdx(self):
        """ """

        numLaneSections = len(self.lane_sections)

        if numLaneSections > 1:
            return numLaneSections - 1

        return 0
    
    def calcLaneOffset(self, s_pos: float):
        """Calculate the lane offset at given position.
        
        Args:
            s_pos:

        Returns:
            offset: float, offset in meters from the center line
        
        Notes:
            1. the validity of s_pos wasn't checked here.
        """
        idx = -1
        n = len(self.laneOffsets)
        for i in range(1, n):
            if s_pos<self.laneOffsets[i].start_pos:
                idx = i-1
                break
        ds = s_pos if idx==0 or n==1 else s_pos-self.laneOffsets[idx].start_pos
        offset = self.laneOffsets[idx].calc(ds)
        return offset


class LaneOffset(RoadRecord):
    """The lane offset record defines a lateral shift of the lane reference line
    (which is usually identical to the road reference line).

    (Section 5.3.7.1 of OpenDRIVE 1.4)

    """
    def calc(self, ds: float):
        offset = Poly3.calc_function_value(ds, self.polynomial_coefficients)
        return offset


class LeftLanes:
    """ """

    sort_direction = False

    def __init__(self):
        self._lanes = []

    @property
    def lanes(self):
        """ """
        self._lanes.sort(key=lambda x: x.id, reverse=self.sort_direction)
        return self._lanes


class CenterLanes(LeftLanes):
    """ """


class RightLanes(LeftLanes):
    """ """

    sort_direction = True


class Lane:
    """ """

    laneTypes = [
        "none",
        "driving",
        "stop",
        "shoulder",
        "biking",
        "sidewalk",
        "border",
        "restricted",
        "parking",
        "bidirectional",
        "median",
        "special1",
        "special2",
        "special3",
        "roadWorks",
        "tram",
        "rail",
        "entry",
        "exit",
        "offRamp",
        "onRamp",
    ]

    def __init__(self, parentRoad, lane_section):
        self._parent_road = parentRoad
        self._id = None
        self._type = None
        self._level = None  # "True" or "False"
        self._link = LaneLink()
        self._widths = []
        self._borders = []
        self.lane_section = lane_section
        self.has_border_record = False

    @property
    def parentRoad(self):
        """ """
        return self._parent_road

    @property
    def id(self):
        """ """
        return self._id

    @id.setter
    def id(self, value):
        self._id = int(value)

    @property
    def type(self):
        """ """
        return self._type

    @type.setter
    def type(self, value):
        if value not in self.laneTypes:
            raise Exception()

        self._type = str(value)

    @property
    def level(self):
        """ """
        return self._level

    @level.setter
    def level(self, value):
        if value not in ["true", "false"] and value is not None:
            raise AttributeError("Value must be true or false.")

        self._level = value == "true"

    @property
    def link(self):
        """ """
        return self._link

    @property
    def widths(self):
        """ """
        self._widths.sort(key=lambda x: x.start_offset)
        return self._widths

    @widths.setter
    def widths(self, value):
        """"""
        self._widths = value

    def getWidth(self, widthIdx):
        """

        Args:
          widthIdx:

        Returns:

        """
        for width in self._widths:
            if width.idx == widthIdx:
                return width

        return None

    def getLastLaneWidthIdx(self):
        """Returns the index of the last width sector of the lane"""

        numWidths = len(self._widths)

        if numWidths > 1:
            return numWidths - 1

        return 0

    @property
    def borders(self):
        """ """
        return self._borders
    
    def calcWidth(self, s_pos: float):
        """
        
        Args:
            s_pos: position from the entry of the lane section.

        Returns:
            w: float

        Notes:
            1. the validity of s_pos wasn't checked.
        """
        n = len(self.widths)
        if n == 0:
            return 0.0
        
        # find the LaneWidth in which s_pos locate
        idx = -1
        for i in range(1, n):
            if s_pos<self.widths[i].start_offset:
                idx = i-1
                break
        ds = s_pos if idx==0 or n==1 else s_pos-self.widths[idx].start_offset
        w = self.widths[idx].calc(ds)
        return w

class LaneLink:
    """ """

    def __init__(self):
        self._predecessor = None
        self._successor = None

    @property
    def predecessorId(self):
        """ """
        return self._predecessor

    @predecessorId.setter
    def predecessorId(self, value):
        self._predecessor = int(value)

    @property
    def successorId(self):
        """ """
        return self._successor

    @successorId.setter
    def successorId(self, value):
        self._successor = int(value)


class LaneSection:
    """The lane section record defines the characteristics of a road cross-section.

    (Section 5.3.7.2 of OpenDRIVE 1.4)

    """

    def __init__(self, road=None):
        self.idx = None
        self.sPos = None
        self.length = None
        self._singleSide = None
        self._leftLanes = LeftLanes()
        self._centerLanes = CenterLanes()
        self._rightLanes = RightLanes()

        self._parentRoad = road

    @property
    def id(self):
        return self.idx

    @property
    def singleSide(self):
        """Indicator if lane section entry is valid for one side only."""
        return self._singleSide

    @singleSide.setter
    def singleSide(self, value):
        if value not in ["true", "false"] and value is not None:
            raise AttributeError("Value must be true or false.")

        self._singleSide = value == "true"

    @property
    def leftLanes(self):
        """Get list of sorted lanes always starting in the middle (lane id -1)"""
        return self._leftLanes.lanes

    @property
    def centerLanes(self):
        """ """
        return self._centerLanes.lanes

    @property
    def rightLanes(self):
        """Get list of sorted lanes always starting in the middle (lane id 1)"""
        return self._rightLanes.lanes

    @property
    def allLanes(self):
        """Attention! lanes are not sorted by id"""
        return self._leftLanes.lanes + self._centerLanes.lanes + self._rightLanes.lanes

    def getLane(self, lane_id: int) -> Lane:
        """

        Args:
          lane_id:

        Returns:

        """
        for lane in self.allLanes:
            if lane.id == lane_id:
                return lane

        return None

    @property
    def parentRoad(self):
        """ """
        return self._parentRoad


class LaneWidth(RoadRecord):
    """Entry for a lane describing the width for a given position.
    (Section 5.3.7.2.1.1.2 of OpenDRIVE 1.4)


    start_offset being the offset of the entry relative to the preceding lane section record

"""

    def __init__(
        self,
        *polynomial_coefficients: float,
        idx: int = None,
        start_offset: float = None
    ):
        self.idx = idx
        self.length = 0
        super().__init__(*polynomial_coefficients, start_pos=start_offset)

    @property
    def start_offset(self):
        """Return start_offset, which is the offset of the entry to the
        start of the lane section.
        """
        return self.start_pos

    @start_offset.setter
    def start_offset(self, value):
        self.start_pos = value

    def calc(self, ds):
        """
        
        Args:
            ds: {float}, is the distance along the road reference line 
            between the start of a new lane offset element and the given position.

        Returns:
            w: {float}, the lane width at that position.

        Notes:
            1. Here, we dont' check whether ds exceeds the valid length of 
            the LaneWidth object.
        """
        w = Poly3.calc_function_value(ds, self.polynomial_coefficients)
        return w
    
    def calcValues(self, ds_array):
        """Similar to function `calc_width()`.
        
        Args:
            ds_array: {array-like}

        Returns:
            w_array: {ndarray}
        """
        if not isinstance(ds_array, np.ndarray):
            ds_array = np.array(ds_array)
        w_array = self.polynomial_coefficients[0] +\
                self.polynomial_coefficients[1] * ds_array +\
                self.polynomial_coefficients[2] * ds_array**2 +\
                self.polynomial_coefficients[3] * ds_array**3
        return w_array


class LaneBorder(LaneWidth):
    """Describe lane by width in respect to reference path.

    (Section 5.3.7.2.1.1.3 of OpenDRIVE 1.4)

    Instead of describing lanes by their width entries and, thus,
    invariably depending on influences of inner
    lanes on outer lanes, it might be more convenient to just describe
    the outer border of each lane
    independent of any inner lanes’ parameters.
    """
