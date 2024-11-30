# -*- coding: utf-8 -*-
# Author: Rongsong Li <rongsong.li@qq.com>
# License: TDG-Attribution-NonCommercial-NoDistrib

import os
import carla
import numpy as np
import enum

from coriskyscene.data_collection.sensors.sensor_position import RSU_RELATIVE_TRANSFORM

def get_camera_intrinsic(sensor):
    """
    Retrieve the camera intrinsic matrix.

    Parameters
    ----------
    sensor : carla.sensor
        Carla rgb camera object.

    Returns
    -------
    matrix_x : np.ndarray
        The 2d intrinsic matrix.

    """
    VIEW_WIDTH = int(sensor.attributes['image_size_x'])
    VIEW_HEIGHT = int(sensor.attributes['image_size_y'])
    VIEW_FOV = int(float(sensor.attributes['fov']))

    matrix_k = np.identity(3)
    matrix_k[0, 2] = VIEW_WIDTH / 2.0
    matrix_k[1, 2] = VIEW_HEIGHT / 2.0
    matrix_k[0, 0] = matrix_k[1, 1] = VIEW_WIDTH / \
        (2.0 * np.tan(VIEW_FOV * np.pi / 360.0))

    return matrix_k


def add_transform(tf_a, tf_b):
    tf_c = carla.Transform(
        tf_a.location + tf_b.location,
        carla.Rotation(
            yaw = tf_a.rotation.yaw + tf_b.rotation.yaw,
            pitch = tf_a.rotation.pitch + tf_b.rotation.pitch,
            roll = tf_a.rotation.roll + tf_b.rotation.roll
        )
    )
    return tf_c

def minus_transform(tf_a, tf_b):
    tf_c = carla.Transform(
        tf_a.location - tf_b.location,
        carla.Rotation(
            yaw = tf_a.rotation.yaw - tf_b.rotation.yaw,
            pitch = tf_a.rotation.pitch - tf_b.rotation.pitch,
            roll = tf_a.rotation.roll - tf_b.rotation.roll
        )
    )
    return tf_c

class CostomTLType(enum.Enum):
    POLE = 1
    RIGHT_ANGLE = 2

def get_traffic_light_type(tl_actor):
    """Judge the type of the given traffic light based onn its extent.
    
    Parameters
    ----------
    tl_actor: CARLA.Actor

    Returns
    -------
    tl_type: str|None
    """
    type_to_extent = {
        CostomTLType.POLE: [1.0, 0.5, 1.0],
        CostomTLType.RIGHT_ANGLE: [7.0, 0.5, 1.0]
    }
    cur_extent = tl_actor.bounding_box.extent
    cur_extent = [cur_extent.x, cur_extent.y, cur_extent.z]
    tl_type = None
    for _type,extent in type_to_extent.items():
        if np.allclose(extent, cur_extent, rtol=0.0, atol=0.05):
            tl_type = _type
            break
    return tl_type

def spawn_point_estimation_using_shape_rsu(parent, sensor_type, relative_position="forward", z_offset=4.2672, fixed_z=None):
    """Get sensor's transform relative to its parent based on parent's shape.

    Parameters
    ----------
    parent: CARLA.Actor
        Currently, the actor is expected to be traffic light.

    sensor_type: str
        One of {"camera", "lidar", "gnss"}.

    relative_positin: str
        One of {"forward", "backward"}

    z_offset: float
        Mounted height in meters relative to its parent. Defaults to 14 feet (4.2672m).

    fixed_z: float
        If set, the sensor will be mounted at a fixed height.

    Returns
    -------
    tf: CARLA.Transform
        The transform  based on which the sensor will be spawned.
    
    Notes
    -----
    1. This is a workaround to attach sensors to traffic lights (serve as 
        road side unit). If you want to regard other types of `parent` (
        e.g., street light) as RSU, write revelant code yourself.
    """
    tl_type = get_traffic_light_type(parent)
    
    extent = parent.bounding_box.extent
    transform = parent.get_transform()
    rel_z = z_offset if fixed_z is None else fixed_z - transform.location.z
    if tl_type == CostomTLType.POLE:  # such traffic light stands on the side of the road
        if sensor_type == 'camera':
            if relative_position == 'forward':  # face the junction
                rel_tf = carla.Transform(
                    carla.Location(z=rel_z),
                    carla.Rotation(yaw=-90, pitch=-15)
                )
            elif relative_position == 'backward':
                rel_tf = carla.Transform(
                    carla.Location(z=rel_z),
                    carla.Rotation(yaw=90, pitch=-15)
                )
            else:
                raise ValueError("Unexpected relative_position: {0}".format(relative_position))
        elif sensor_type == 'lidar':
            rel_tf = carla.Transform(
                carla.Location(z=rel_z)
            )
        elif sensor_type == 'gnss':
            rel_tf == carla.Transform(
                carla.Location(z=rel_z)
            )
        else:
            raise ValueError("Unexpected sensor_type '{0}'".format(sensor_type))
    elif tl_type == CostomTLType.RIGHT_ANGLE:  # such traffic light hang over the road
        if sensor_type == 'camera':
            if relative_position == 'forward':
                rel_tf = carla.Transform(
                    carla.Location(x=-extent.x, z=rel_z),
                    carla.Rotation(yaw=90, pitch=-15)
                )
            elif relative_position == 'backward':
                rel_tf = carla.Transform(
                    carla.Location(x=-extent.x, z=rel_z),
                    carla.Rotation(yaw=-90, pitch=-15)
                )
            else:
                raise ValueError("Unexpected relative_position: {1}".format(relative_position))
        elif sensor_type == 'lidar':
            rel_tf = carla.Transform(
                carla.Location(x=-extent.x, z=rel_z)
            )
        elif sensor_type == 'gnss':
            rel_tf = carla.Transform(
                carla.Location(x=-extent.x, z=rel_z)
            )
        else:
            raise ValueError("Unexpected sensor_type '{0}'".format(sensor_type))
    tf = rel_tf
    return tf

def spawn_point_estimation_rsu(parent, sensor_type, map_name, relative_position="forward"):
    """Get the sensor's spawn point relative to its parent actor
    based on predefined trasform.

    Parameters
    ----------
    parent: CARLA.Actor
        Currently, the actor is expected to be traffic light.

    sensor_type: str
        One of {"camera", "lidar", "gnss"}.

    map_name: str
        Name of the used CARLA map. E.g., it can be "Carla/Maps/Town01" or 
        "Town01".

    relative_positin: str
        One of {"forward", "backward"}

    Returns
    -------
    tf: CARLA.Transform
        The relative transform  based on which the sensor will be spawned.
    """
    assert relative_position=='forward' or relative_position == 'backward', \
        "relative_position is expected to be 'forward' or 'backward', got {}"\
        .format(relative_position)

    map_name = os.path.basename(map_name)
    landmark_id = parent.get_opendrive_id()
    info = RSU_RELATIVE_TRANSFORM[map_name][landmark_id]
    
    if sensor_type == 'camera':
        rel_tf = carla.Transform(
            carla.Location(x=info['x'], y=info['y'], z=info['z']),
            carla.Rotation(yaw=info[relative_position]['yaw'], 
                           pitch=info[relative_position]['pitch'])
        )
    elif sensor_type == 'lidar':
        rel_tf = carla.Transform(
            carla.Location(x=info['x'], y=info['y'], z=info['z'])
        )
    elif sensor_type == 'gnss':
        rel_tf = carla.Transform(
            carla.Location(x=info['x'], y=info['y'], z=info['z'])
        )
    else:
        raise ValueError("Unexpected sensor_type '{0}'".format(sensor_type))
    return rel_tf


def spawn_point_estimation_cav(parent, sensor_type, relative_position="front", z_offset=0.3):
    """Get the sensor's spawn point relative to its parent actor.
    
    Parameters
    ----------
    parent: CARLA.Actor
        The actor to which the sensor will attach to.

    sensor_type: str
        One of {"camera", "lidar", "gnss"}.

    relative_position: str
        One of {"fornt", "rear", "left", "right"}.

    z_offset: float
        The sensor's mounting height (in meters) relative to its parent.

    Returns
    -------
    transform: CARLA.Transform
        Sensor's transform relative to its parent. 
    """
    extent = parent.bounding_box.extent
    if sensor_type == "camera":
        if relative_position == 'front':
            spawn_point = carla.Transform(
                carla.Location(0.1, 0, extent.z*2+z_offset),
                carla.Rotation(yaw=0)
            )
        elif relative_position == 'rear':
            spawn_point = carla.Transform(
                carla.Location(-0.1, 0, extent.z*2+z_offset),
                carla.Rotation(yaw=180)
            )
        elif relative_position == 'left':
            spawn_point = carla.Transform(
                carla.Location(0, -0.1, extent.z*2+z_offset),
                carla.Rotation(yaw=-90)
            )
        elif relative_position == 'right':
            spawn_point = carla.Transform(
                carla.Location(0, 0.1, extent.z*2+z_offset),
                carla.Rotation(yaw=90)
            )
        else:
            raise ValueError("Unexpected camera's relative_position: '{0}'".format(relative_position))
    elif sensor_type == "lidar":
        spawn_point = carla.Transform(
                carla.Location(0, 0, extent.z*2+z_offset),
                carla.Rotation(0,0,0)
            )
    elif sensor_type == "gnss":
        spawn_point = carla.Transform(
            carla.Location(-0.2, 0, extent.z*2)
        )
    else:
        raise ValueError("Unexpected sensor_type '{0}'".format(sensor_type))
    return spawn_point