# -*- coding: utf-8 -*-
'''
@File    :   roadObjects.py
@Time    :   2024-01-10
@Author  :   Rongsong Li <rongsong.li@qq.com>
@Version :   1.0
@Desc    :   definitions for <objects> element in <road> element in 
             OpenDRIVE file.
@TODO    :   add definition of <outline>
'''

class RoadObjects:
    """The definition of <objects> element in <road> element 
    in OpenDRIVE file. 

    (Section 11 in OpenDRIVE 1.6)
    
    """
    def __init__(self) -> None:
        self._objects = []

    @property
    def objects(self):
        self._objects.sort(key=lambda x: x.origin_s_pos)
        return self._objects
    
    def getObject(self, object_id):
        for obj in self._objects:
            if obj.id == object_id:
                return obj 
        return None

class RoadOjbect:
    """The definition of <object> element in <objects> element.
    
    (Section 11 in OpenDRIVE 1.6)
    """
    objectTypes = [
        "crosswalk"
    ]
    def __init__(self) -> None:
        self._origin_s_pos = None
        self._origin_t_pos = None
        self._z_offset = None
        self._type = None
        self._valid_length = None
        self._orientation = None
        self._subtype = None
        self._dynamic = False  # bool
        self._heading = None
        self._pitch = None
        self._roll = None
        self._name = None
        self._id = None
        self._height = None
        self._length = None
        self._width = None
        self._radius = None

    @property
    def origin_s_pos(self):
        """s-coordinate of object's origin"""
        return self._origin_s_pos
    
    @origin_s_pos.setter
    def origin_s_pos(self, value: float):
        self._origin_s_pos = value

    @property
    def origin_t_pos(self):
        """t-coordinate of object's origin"""
        return self._origin_t_pos
    
    @origin_t_pos.setter
    def origin_t_pos(self, value: float):
        self._origin_t_pos = value
    
    @property
    def z_offset(self):
        """z-offset of object's origin relative to the elevation of the reference line"""
        return self._z_offset
    
    @z_offset.setter
    def z_offset(self, value: float):
        self._z_offset = value

    @property
    def type(self):
        return self._type
    
    @type.setter
    def type(self, value):
        if value not in self.objectTypes:
            raise ValueError("Unexpected object type '{0}', "
                             "the following is supported: {1}"
                             .format(self.objectTypes))
        self._type = str(value)

    @property
    def heading(self):
        """Heading angle of the object relative to road direction"""
        return self._heading
    
    @heading.setter
    def heading(self, value: float):
        self._heading = value

    @property
    def pitch(self):
        """Pitch angle relative to the x/y-plane"""
        return self._pitch
    
    @pitch.setter
    def pitch(self, value: float):
        self._pitch = value

    @property
    def roll(self):
        """Roll angle relative to the x/y-plane"""
        return self._roll
    
    @roll.setter
    def roll(self, value:float):
        self._roll = value

    @property
    def orientation(self):
        """ 
        "" = valid in positive s-direction, "-" = valid in negative s-direction,
        + "none" = valid in both directions+ (does not affect the heading)
        """
        return self._orientation
    
    @orientation.setter
    def orientation(self, value: str):
        self._orientation = value

    @property
    def dynamic(self):
        """Indicates whether the object is dynamic or static, default to no."""
        return self._dynamic
    
    @dynamic.setter
    def dynamic(self, value: bool):
        self._dynamic = value

    @property
    def id(self):
        return self._id
    
    @id.setter
    def id(self, value):
        self._id = int(value)

    @property
    def name(self):
        return self._name
    
    @name.setter
    def name(self, value):
        self._name = value

    @property
    def width(self):
        """Width of the angular object’s bounding box, alternative to @radius"""
        return self._width
    
    @width.setter
    def width(self, value: float):
        self._width = value

    @property
    def length(self):
        """Length of the object’s bounding box, alternative to @radius"""
        return self._length
    
    @length.setter
    def length(self, value: float):
        self._length = value

    @property
    def radius(self):
        """Radius of the circular object's bounding box, alternative to @length"""
        return self._radius
    
    @radius.setter
    def radius(self, value: float):
        self._radius = value

    @property
    def valid_length(self):
        """Validity of object along s-axis (0.0 for point object)"""
        return self._valid_length
    
    @valid_length.setter
    def valid_length(self, value: float):
        self._valid_length = value
