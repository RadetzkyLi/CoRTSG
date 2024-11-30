#!/usr/bin/env python
# -*- coding: utf-8 -*-
'''
@File    :   occlusion_model.py
@Date    :   2023-12-20
@Author  :   Rongsong Li <rongsong.li@qq.com>
@Version :   1.0
@Desc    :   Composition of the ray-casting based occlusion model,
            including geometry utility functions, geometry definition,
            occlusion judgement functions.
'''

import numpy as np
import itertools


# ===========================================================
# - 2D Geometry Utility Functions
# ===========================================================
def calc_distance_2d(p1, p2):
    """"Compute Euclidean distance between two 2d points."""
    d = ((p1[0]-p2[0])**2 + (p1[1]-p2[1])**2)**0.5
    return d

def get_vector_2d(p1, p2):
    """Get a 2d vector pointing to p2 from p1"""
    return [p2[0]-p1[0], p2[1]-p1[1]]

def calc_cross_product_2d(vec1, vec2):
    """Compute cross product for two 2d vectors."""
    # p1.x*p2.y - p2.x*p1.y
    res = vec1[0]*vec2[1] - vec1[1]*vec2[0]
    return res

def calc_dot_product_2d(vec1, vec2):
    return vec1[0]*vec2[0] + vec1[1]*vec2[1]

def calc_length_2d(vec):
    return (vec[0]**2 + vec[1]**2)**0.5

def calc_cosine_of_vectors(vec1, vec2):
    len1,len2 = calc_length_2d(vec1),calc_length_2d(vec2)
    epsilon = 1e-8
    if len1<=epsilon or len2<=epsilon:
        return 0.0
    val = calc_dot_product_2d(vec1, vec2)/(len1*len2)
    return val

def calc_triangle_area(pa, pb, pc):
    """Compute the area of triangle composed of point A, B and C.
    area = |AB x AC|/2.0

    Parameters
    ----------
    pa, pb, pc :
        2d location.

    Returns
    -------
    val : float

    """
    vec_ab = get_vector_2d(pa, pb)
    vec_ac = get_vector_2d(pa, pc)
    val = abs(calc_cross_product_2d(vec_ab, vec_ac))*0.5
    return val

def is_point_in_triangle(pa, pb, pc, pd):
    """Judge whether the given point D is lying in the triangle 
    composed by point A, B and C.
    
              A
              /\ 
             /  \  * D
            /    \ 
        B  +------+ C
    
    Parameters
    ----------
    pa, pb, pc, pe:
        2d location.

    Returns
    -------
    flag : bool
        True if D lies in the triangle (including on the edge)
    """
    # first: judge whether point D lie in Axis Align Bounding Box (AABB)
    min_x = min(pa[0], pb[0], pc[0])
    max_x = max(pa[0], pb[0], pc[0])
    min_y = min(pa[1], pb[1], pc[1])
    max_y = max(pa[1], pb[1], pc[1])
    if not (pd[0]>=min_x and pd[0]<=max_x and pd[1]>=min_y and pd[1]<=max_y):
        return False
    
    # second: judge whether point D lie in triangle
    tri_area = calc_triangle_area(pa, pb, pc)
    split_tri_area = calc_triangle_area(pa, pb, pd) +\
                    calc_triangle_area(pa, pc, pd) +\
                    calc_triangle_area(pd, pb, pc)
    epsilon = 0.05
    if abs(tri_area-split_tri_area) < epsilon:
        return True
    return False

def is_point_in_rectangle(pa, pb, pc, pd, pe):
    """Judge whether the given point E is lying in the rectangle
    composed by linking A,B,C and D clockwise.

     (A)+--------------------+(B)
        |                    |
        |        *(E)        |
        +--------------------+
        (D)                  (C)

    Parameters
    ----------
    pa, pb, pc, pd: 
        2d location.

    pe: 
        2d location.

    Returns
    -------
    flag: bool
        True if E lies in the racangle.

    Notes
    -----
    1. Boundary point is also taken as in rectangle.
    """
    # first: judge whether point E lie in Axis Align Bounding Box (AABB)
    min_x = min(pa[0], pb[0], pc[0], pd[0])
    max_x = max(pa[0], pb[0], pc[0], pd[0])
    min_y = min(pa[1], pb[1], pc[1], pd[1])
    max_y = max(pa[1], pb[1], pc[1], pd[1])
    if not (pe[0]>=min_x and pe[0]<=max_x and pe[1]>=min_y and pe[1]<=max_y):
        return False

    # second: judge whether point E lie in Oriented Bounding Box (OBB)
    vec_ab = get_vector_2d(pa, pb)
    vec_ae = get_vector_2d(pa, pe)
    vec_cd = get_vector_2d(pc, pd)
    vec_ce = get_vector_2d(pc, pe)
    
    vec_da = get_vector_2d(pd, pa)
    vec_de = get_vector_2d(pd, pe)
    vec_bc = get_vector_2d(pb, pc)
    vec_be = get_vector_2d(pb, pe)
    flag = (calc_cross_product_2d(vec_ab, vec_ae) * calc_cross_product_2d(vec_cd, vec_ce) >= 0) &\
            (calc_cross_product_2d(vec_da, vec_de) * calc_cross_product_2d(vec_bc, vec_be) >= 0)
    return flag

def is_point_in_circular_sector(po, pa, pb, r, pc):
    """Judge whether the given point C lies in circular
    sector AOB, whose vectorial angle is less than 180 degrees.

        /A
       /     *C
      /_______B
      O

    Parameters
    ----------
    po,pa,pb:
        2d coordinates of point O,A,B.
    r: float
        The radiu of the circular sector.

    pc:
        2d coordinates of point C.

    Returns
    -------
    flag:
        True if point C lies in circular sector AOB. 

    Notes
    -----
    1. Boundary points are also taken in the circular sector.
    """
    vec_oa = get_vector_2d(po, pa)
    vec_ob = get_vector_2d(po, pb)
    vec_oc = get_vector_2d(po, pc)

    u = calc_cross_product_2d(vec_oc, vec_oa) / calc_cross_product_2d(vec_ob, vec_oa)
    v = calc_cross_product_2d(vec_oc, vec_ob) / calc_cross_product_2d(vec_oa, vec_ob)
    
    d = calc_distance_2d(po, pc)
    if (d<=r) & (u>=0.0) & (v>=0.0):
        flag = True
    else:
        flag = False

    return flag

def get_circular_sector_vertices_from_polygon(po, center, vertices):
    """Given a covex polygon and point O out of it,
    find two vertices, A and D, so that the resulting circular 
    sector AOD has the maximal radius angle. 
    
    E.g., in the following figure, /_AOD is the maximal
    radius angle. 

                  A ________ B
                    |       |
                  C |_______|D
            * O

    Parameters
    ----------
    po: array-like
        The coordinate of point O, of shape (2,).

    center: array-like
        The coordinate of the polygen, of shape (2,).

    vertices: array-like
        The coordinates of the polygon's vertices, of shape (n,2).

    Returns
    -------
    points: list
        Two vertices' 2D coordinates, of shape (2,2)
    """
    combs = list(itertools.combinations(vertices, 2))
    cos_vals = [calc_cosine_of_vectors(get_vector_2d(po, comb[0]), get_vector_2d(po, comb[1]))
                for comb in combs]
    indices = np.argsort(cos_vals)
    target_index = indices[0]
    return [combs[target_index][0], combs[target_index][1]]

def calc_line_segment_intersection(pa, pb, pc, pd):
    """Compute the interseciton of two line segments.

    Parameters
    ----------
    pa,pb :
        two points on line1
    
    pc,pd : 
        two points on line2

    Returns
    -------
    pe : 
        coordinates of the intersection. Returns None if the two line
        segments have no intersection or are parallel.

    Refs
    ----
    1. https://zhuanlan.zhihu.com/p/598112630
    """
    vec_ab = get_vector_2d(pa, pb)
    vec_cd = get_vector_2d(pc, pd)

    det = calc_cross_product_2d(vec_cd, vec_ab)

    eps = 1e-8
    # the two are parallel
    if abs(det) <= eps:
        return None
    
    vec_ac = get_vector_2d(pa, pc)
    t = calc_cross_product_2d(vec_cd, vec_ac)/det
    u = calc_cross_product_2d(vec_ab, vec_ac)/det

    if t>-eps and t<1+eps and u>-eps and u<1+eps:
        pe = [pa[0] + t*vec_ab[0], pa[1] + t*vec_ab[1]]
        return pe
    
    return None



# ================================================================
# - Geometry Class Definition
# ================================================================
def x_to_world_transformation(transform):
    """"
    Obtain the matrix from local to global. It is a 2d variant of 
    that of CARLA.
    
    """
    rotation = transform.rotation
    location = transform.location

    # used for rotation matrix
    c_y = np.cos(np.radians(rotation.yaw))
    s_y = np.sin(np.radians(rotation.yaw))

    matrix = np.identity(3)
    # translation matrix
    matrix[0, 2] = location.x 
    matrix[1, 2] = location.y 

    # rotation matrix
    matrix[0, 0] = c_y
    matrix[0, 1] = -s_y
    matrix[1, 0] = s_y
    matrix[1, 1] = c_y

    return matrix

class Location:
    "location in meters"
    def __init__(self, x:float=0.0, y:float=0.0) -> None:
        self.x = x
        self.y = y

    def __repr__(self) -> str:
        return "Location(x={0},y={1})".format(self.x, self.y)
    
    def distance(self, other_location):
        d = ((self.x-other_location.x)**2 +
             (self.y-other_location.y)**2)**0.5
        return d
    
    def __eq__(self, other) -> bool:
        """Returns True if both locations are the same point in space."""
        epsilon = 0.02 # meters
        return np.allclose(self.to_list(), other.to_list(), rtol=0.0, atol=epsilon)
    
    def set_location(self, x:float, y:float):
        self.x = x
        self.y = y
        return self
    
    def to_list(self):
        return [self.x, self.y]

class Vector2D:
    def __init__(self, x:float=0.0, y:float=0.0) -> None:
        self.x = x  # in meters
        self.y = y

    def __repr__(self) -> str:
        return "Vector2D(x={0},y={1})".format(self.x, self.y)
    
    def dot(self, vector):
        return self.x*vector.x + self.y*vector.y
    
    def to_list(self):
        return [self.x, self.y]

class Rotation:
    def __init__(self, yaw:float=0) -> None:
        self.yaw = yaw  # in degree

    def __repr__(self) -> str:
        return "Rotation(yaw={0})".format(self.yaw)

class Transform:
    def __init__(self, location, rotation) -> None:
        self.location = location
        self.rotation = rotation

    def get_matrix(self):
        return x_to_world_transformation(self)

class BoundingBox:
    def __init__(self, extent, location, rotation=None, height:float=0.0) -> None:
        """Definition of an Oriented Bounding Box.
        
        Parameters
        ----------
        extent: Vector2D
            The extent (half length) of the bbox.

        location: Location
            The 2d coordinate of the center.

        rotation: Rotation
            Currently, only include the yaw in degrees of the bbox. The 
            yaw denotes the angle to the x-axis.
        
        height: float
            The height of the bounding box.
        """
        self.location = location
        self.extent = extent
        self.rotation = Rotation() if rotation is None else rotation
        self.height = height
        # calc the world vertices based on location and rotation of itself
        self.vertices = self.get_world_vertices(
            Transform(self.location, self.rotation),
            True
        )
        # get axes
        self.axes = self.get_axes()

    def __repr__(self) -> str:
        return "BoundingBox(extent={0}, location={1}, rotation={2}, height={3})".format(
            self.extent, self.location, self.rotation, self.height
        )

    def set_location(self, x:float, y:float):
        self.location = Location(x, y)
        self.update_vertices()
        return self
    
    def set_transform(self, x: float, y: float, yaw: float):
        # yaw in degrees
        self.location = Location(x, y)
        self.rotation = Rotation(yaw)
        self.update_vertices()
        return self

    def get_local_vertices(self):
        """start from left up corner and in clockwise, i.e., 
        0 --> 1 --> 2 --> 3 as the following:
        
           0 +---------+ 1
             |         |
             |         |
           3 +---------+ 2
        
        """
        x,y = self.extent.x,self.extent.y
        vertices = [
            Location(x, -y),
            Location(x, y),
            Location(-x, y),
            Location(-x, -y)
        ]
        return vertices

    def get_world_vertices(self, transform, return_array=False):
        """Obtain the vertices of the bbox.
        For case of 2d, 4 vertices is expected.
        """
        x,y = self.extent.x,self.extent.y
        local_vertices = np.array([
            [x, -y, 1],
            [x, y, 1],
            [-x, y, 1],
            [-x, -y, 1]
        ])
        mat = np.array(transform.get_matrix())
        global_vertices = np.matmul(mat, np.transpose(local_vertices))
        global_vertices = np.transpose(global_vertices)
        v = global_vertices
        if return_array:
            return v[:, :2]
        global_vertices = [
            Location(v[0][0], v[0][1]),
            Location(v[1][0], v[1][1]),
            Location(v[2][0], v[2][1]),
            Location(v[3][0], v[3][1])
        ]
        return global_vertices
    
    def update_vertices(self):
        self.vertices = self.get_world_vertices(
            Transform(self.location, self.rotation),
            True
        )
    
    def get_axes(self):
        """Get the two axes along the x and y direction."""
        s = np.sin(np.radians(self.rotation.yaw))
        c = np.cos(np.radians(self.rotation.yaw))
        return [
            Vector2D(c, s),
            Vector2D(-s, c) 
        ]
    
    def get_projection_radius(self, axis):
        return self.extent.x * np.abs(axis.dot(self.axes[0])) +\
                self.extent.y * np.abs(axis.dot(self.axes[1]))
    
    def get_two_vertices_for_circular_sector(self, po):
        """
        
        Parameters
        ----------
        po : Location
            The coordinate of point O.

        Returns
        -------
        points : list[Location]
        """
        points = get_circular_sector_vertices_from_polygon(
            [po.x, po.y],
            [self.location.x, self.location.y],
            self.vertices
        )
        points = [
            Location(points[0][0], points[0][1]),
            Location(points[1][0], points[1][1])
        ]
        return points
    
    def intersects(self, bbox):
        """
        Judge whether two bbox intersects.

        Parameters
        ----------
        bbox: BoundingBox
            Another bbox.

        Returns
        -------
        flag: bool
            Return True if they intersects.
        
        Refs
        ----
        1. https://blog.csdn.net/ever_crazy/article/details/106699193
        """
        nv = Vector2D(
            x = self.location.x - bbox.location.x,
            y = self.location.y - bbox.location.y
        )
        proj_axis_list = [self.axes[0], self.axes[1], 
                          bbox.axes[0], bbox.axes[1]]
        for axis in proj_axis_list:
            if (self.get_projection_radius(axis) + bbox.get_projection_radius(axis))\
                <= np.abs(nv.dot(axis)):
                return False
        return True
    
    def contains(self, point):
        """Returns True if a point passed in world space is inside this bounding box.
        
        Parameters
        ----------
        point : Location
            The point in world space to be checked

        Returns
        -------
        flag : bool
        
        """
        flag = is_point_in_rectangle(
            self.vertices[0],
            self.vertices[1],
            self.vertices[2],
            self.vertices[3],
            point.to_list()
        )
        return flag



# ==========================================================
# - Occlusion Judgement Functions
# ==========================================================
def is_points_in_circular_sector_bbox(ego, occluder, point_list, radius=120.0):
    """Judge the point in point_list locates in curcular section.
    
    Parameters
    ----------
    ego : BoundingBox

    occluder : BoundingBox

    point_list : array-like
        2d location array.

    radius : float
        The radius of the circular sector.

    Returns
    -------
    res : list
        Same length as point_list
    """
    assert radius > 0.0

    two_vertices = occluder.get_two_vertices_for_circular_sector(ego.location)
    loc_ego = [ego.location.x, ego.location.y]
    loc_ver_1 = [two_vertices[0].x, two_vertices[0].y]
    loc_ver_2 = [two_vertices[1].x, two_vertices[1].y]
    flag_list = [is_point_in_circular_sector(
        loc_ego, loc_ver_1, loc_ver_2, radius, point
    ) for point in point_list]
    return flag_list

def calc_occlusion_height(h_ego, h_occluder, delta_h, d_occluder, d_occludee):
    """Calculate the occlusion height, below which, object is invisible.
    
    Parameters
    ----------
    h_ego: float
        The height in meters of the ego vehicle
    h_occluder: float
        The height in meters of the npc vehicle/object
    delta_h: float
        The mounting height in meters of the LiDAR
    d_occluder: float
        The distance between the ego and the occluder.
    d_occludee: float
        The distance between the ego and the occludee. It should
        be greater than `d_occluder`.

    Returns
    -------
    h_occlusion: float
        The occluded height. Negative value means occludee is not 
        occluded at all.

    Notes
    -----
    1. Currently, all vehicles and obstacles are assumed to be on
    the same plane.
    """
    thr = 0.5
    # occluder should be closer to ego than occludee by some margin
    if d_occludee <= d_occluder + thr:
        return 0.0
    
    h_occlusion = delta_h+h_ego - d_occludee/d_occluder*(delta_h+h_ego-h_occluder)
    return h_occlusion

def is_occluded(ego, occluder, target, mounting_height=0.3, two_vertices=None):
    """If an object locate in the occluded circular section,
    and its occluded ratio (in height direction) exceed thr,
    then the target object is regarded as occludee.

    Parameters
    ----------
    ego : BoundingBox

    occluder : BoundingBox

    target : BoundingBox

    mounting_height : float
        LiDAR's mounting height in meters. The LiDAR is supposed to be 
        mounted at the top of a car.

    two_vertices : list
        Each element is Location.
    
    Returns
    -------
    flag : bool
        Returns True if ego is occluded by occluder when looking at occludee.
    """
    thr = 0.3
    perception_range = 120 # meters

    if two_vertices is None:
        two_vertices = occluder.get_two_vertices_for_circular_sector(ego.location)
    
    # first: calculate the intersection between ego-occludee-line and occluder's bounding box
    p_intersection = calc_line_segment_intersection(
        ego.location.to_list(), target.location.to_list(),
        two_vertices[0].to_list(), two_vertices[1].to_list()
    )
    if p_intersection is None:
        return False
    
    # second: calculate the occluded ratio in height direction
    d_occluder = calc_distance_2d(ego.location.to_list(), p_intersection)
    d_target = ego.location.distance(target.location)
    h_occluded = calc_occlusion_height(
        ego.height,
        occluder.height,
        mounting_height,
        d_occluder,
        d_target
    )

    # third: if occlusion ratio exceeds `thr``, the object is taken as occludee
    ratio = h_occluded/target.height
    if ratio >= thr:
        return True
    return False

