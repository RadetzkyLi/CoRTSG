# -*- coding: utf-8 -*-

import numpy as np

from coriskyscene.opendriveparser.elements.road_record import RoadRecord
from coriskyscene.opendriveparser.elements.geometry import Poly3

__author__ = "Benjamin Orthen, Stefan Urban"
__copyright__ = "TUM Cyber-Physical Systems Group"
__credits__ = ["Priority Program SPP 1835 Cooperative Interacting Automobiles"]
__version__ = "1.0.2"
__maintainer__ = "Benjamin Orthen"
__email__ = "commonroad-i06@in.tum.de"
__status__ = "Released"


class ElevationRecord(RoadRecord):
    """The elevation record defines an elevation entry at a given reference line position.

    (Section 5.3.5.1 of OpenDRIVE 1.4)
    """

class ElevationProfile:
    """The elevation profile record contains a series of elevation records
    which define the characteristics of
    the road's elevation along the reference line.

    (Section 5.3.5 of OpenDRIVE 1.4)
    """

    def __init__(self):
        self.elevations = []
        self._elev_lengths = []  # length of each <elevation> element

    def addElevationRecord(self, elevationRecord: ElevationRecord):
        self.elevations.append(elevationRecord)

    def calcElevationLengths(self, roadLength: float):
        """Calculate the accumulated length. It should be called when all the 
        elevation records are added.

        Args:
            roadLength: float,
        """
        self.elevations.sort(key=lambda x: x.start_pos)
        n = len(self.elevations)
        length = 0
        for i in range(1,n):
            length += self.elevations[i].start_pos
            self._elev_lengths.append(length)
        self._elev_lengths.append(roadLength)
        self._elev_lengths = np.array(self._elev_lengths)

    def calc(self, s_pos: float):
        """
        Args:
            s_pos: float, position in the planeView.

        Returns:
            elev: float, elevation in meters at given position

        Notes:
            1. Here, we don't check whether s_pos exceeds the valid length of the
            elevation profile.
        """
        idx = -1  # index of current <elevation> element
        ds = s_pos
        for i,length in enumerate(self._elev_lengths):
            if s_pos<length:
                idx = i 
                break
        ds = s_pos if idx==0 or len(self._elev_lengths)==1 else s_pos-self._elev_lengths[idx-1]

        elev = Poly3.calc_function_value(ds, self.elevations[idx].polynomial_coefficients)
        return elev