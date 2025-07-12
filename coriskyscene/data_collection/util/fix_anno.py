#!/usr/bin/env python
# -*- coding: utf-8 -*-
'''
@File    :   fix_anno.py
@Date    :   2025-05-31
@Author  :   Rongsong Li <rongsong.li@qq.com>
@Version :   1.0
@Desc    :   Fix bugs in annotations.
'''

import os
import glob
import collections
from tqdm import tqdm

import numpy as np
import carla
from joblib import Parallel,delayed

from coriskyscene.common_utils.fileio import load_yaml, save_yaml


CAMERA_NAMES = ["cameraFront", "cameraRear", "cameraLeft", "cameraRight", "cameraForward", "cameraBackward"]

# =======================================================
# - common utility
# =======================================================

def deep_update_dict(d, u):
    """d: old dict
    u: new dict"""
    for k, v in u.items():
        if isinstance(v, collections.abc.Mapping):
            d[k] = deep_update_dict(d.get(k, {}), v)
        else:
            d[k] = v
    return d

def get_frame_from_path(path, to_int=False):
    fname = os.path.basename(path)
    fname = fname.split(".")[0]
    frame = fname.split("_")[0]
    if to_int:
        frame = int(frame)
    return frame

def get_agent_from_path(path, to_int=True):
    agent = os.path.basename(os.path.dirname(path))
    if agent.startswith("cav"):
        agent = int(agent.split("cav_")[1])
    elif agent.startswith("rsu"):
        agent = int(agent.split("rsu_")[1])
    else:
        raise ValueError("Unknown agent '{0}'".format(agent))
    return agent

def get_agent_from_name(agent_name):
    if agent_name.startswith("cav"):
        agent = int(agent_name.split("cav_")[1])
    elif agent_name.startswith("rsu"):
        agent = int(agent_name.split("rsu_")[1])
    else:
        raise ValueError("Unknown agent '{0}'".format(agent_name))
    return agent

def pose_to_carla_transform(pose):
    """From pose in carla world coordinate (x,y,z,roll,yaw,pitch)
    to carla.Transform."""
    return carla.Transform(
        carla.Location(pose[0], pose[1], pose[2]),
        carla.Rotation(roll=pose[3], yaw=pose[4], pitch=pose[5])
    )

# =======================================================
# - Fix Annotation
# =======================================================
def update_camera_extrinsic(anno_path:str, save_path:str):
    """Update the extrinsic matrix of camera (from camera to LiDAR)
    to the correct one.
    
    Parameters
    ----------
    anno_path : str
        The path of an annotaion file (yaml file).
    save_path : str
        The path to save updated annotation.

    Returns
    -------
    result : dict
        E.g., {anno_path: is_success}
    
    Notes
    -----
    1. **Bug description**: in datasets of `Multi-V2X` and `CoRTSG`, the extrinsic matrix of a camera 
        represents the transformation from LiDAR to camera, calculated as `C = AB`, 
        where A denotes transformation from world to camera and B denotes transformation from
        lidar to world. 
        However, during saving annotation, `C = AB` was mistaken as `C = BA`.
    2. **Bug effects**: if you use camera's extrinsic in annotation file, the results are wrong. 
    3. **Bug fixing**: recalculate extrinsic of the camera using poses and update the annotation file
        accordingly. 
    4. **Fix results**: The bug has been fixed in code of data collection and datasets of `Multi-V2X`
        and `CoRTSG` in 2025-06-01. If you have downloaded the two datasets, to repair it, download the fixed
        annotations from OpenDataLab and replace the old ones.
    """
    anno = load_yaml(anno_path)
    lidar_transform = pose_to_carla_transform(anno['lidar_pose'])
    lidar_to_world = np.array(lidar_transform.get_matrix())

    for camera_name in CAMERA_NAMES:
        if camera_name not in anno:
            continue
        # calculate the correct extrinsic
        camera_transform = pose_to_carla_transform(anno[camera_name]['coords'])
        world_to_camera = np.array(camera_transform.get_inverse_matrix())
        lidar_to_camera = np.dot(world_to_camera, lidar_to_world)

        # update the annotation
        anno[camera_name]['extrinsic'] = lidar_to_camera.tolist()

    # save updated annotation
    save_yaml(anno, save_path)

    return True

def update_camera_extrinsic_in_dataset(dataset_root:dir, save_root:dir):
    """
    Parameters
    ----------
    dataset_root : str
        The root dir of dataset.
    save_root : str
        The root to save the updated annotations. 
    """
    cnt_total = 0
    for scenario_name in os.listdir(dataset_root):
        scenario_dir = os.path.join(dataset_root, scenario_name)
        cnt = 0
        for agent_name in os.listdir(scenario_dir):
            if not (agent_name.startswith("cav") or
                    agent_name.startswith("rsu")):
                continue
            pattern = os.path.join(scenario_dir, agent_name, "*.yaml")
            yaml_paths = glob.glob(pattern)
            yaml_paths = [x for x in yaml_paths if not x.endswith("_view.yaml")]

            # create save paths
            save_dir = os.path.join(save_root, scenario_name, agent_name)
            if not os.path.exists(save_dir):
                os.makedirs(save_dir)
            save_paths = [os.path.join(save_dir, os.path.basename(x)) for x in yaml_paths]
            res = Parallel(n_jobs=16, backend="multiprocessing", verbose=0)(
                delayed(update_camera_extrinsic)(x, y)
                for x,y in zip(yaml_paths, save_paths)
                )
            cnt += np.sum(res)
        print("{0} {1} done! {2} yaml was modified!{0}".format("*"*10, scenario_name, cnt))
        cnt_total += cnt
    print("{0} yaml files were modified!".format(cnt_total))


def update_camera_extrinsic_in_scenario(scenario_root:str, scenario_save_root:str):
    """Work for CoRTSG bacause few timestamps in a scenario"""
    cnt = 0
    for agent_name in os.listdir(scenario_root):
        if not (agent_name.startswith("cav") or
                agent_name.startswith("rsu")):
            continue
        pattern = os.path.join(scenario_root, agent_name, "*.yaml")
        yaml_paths = glob.glob(pattern)
        yaml_paths = [x for x in yaml_paths if not x.endswith("_view.yaml")]

        # create save paths
        save_dir = os.path.join(scenario_save_root, agent_name)
        if not os.path.exists(save_dir):
            os.makedirs(save_dir)
        save_paths = [os.path.join(save_dir, os.path.basename(x)) for x in yaml_paths]
        
        for yaml_path,save_path in zip(yaml_paths, save_paths):
            update_camera_extrinsic(yaml_path, save_path)

        cnt += len(yaml_paths)

    return cnt

def update_camera_extrinsic_in_cortsg(dataset_root:str, save_root:dir):
    cnt_total = 0
    for logical_scenario in os.listdir(dataset_root):
       logical_scenario_root = os.path.join(dataset_root, logical_scenario)
       concrete_scenario_roots = [os.path.join(logical_scenario_root, x) 
                                  for x in os.listdir(logical_scenario_root)]
       concrete_scenario_save_roots = [os.path.join(save_root, logical_scenario, x)
                                       for x in os.listdir(logical_scenario_root)]
       res = Parallel(n_jobs=8, backend="multiprocessing")(delayed(update_camera_extrinsic_in_scenario)
                                                            (x, y) for x,y in 
                                                            zip(concrete_scenario_roots,
                                                                concrete_scenario_save_roots))
       cnt = np.sum(res)
       print("{0} {1} done! {2} yaml was modified!{0}".format("*"*10, logical_scenario, cnt))
       cnt_total += cnt
    print("{0} yaml files were modified!".format(cnt_total))

# =======================================================
# - Test
# =======================================================
def is_reasonable_extrinsic(anno_path:str):
    """If the translation vector exceeds threshold, it seems as abnorml."""
    thr = 0.5  # meters
    anno = load_yaml(anno_path)
    for camera_name in CAMERA_NAMES:
        if camera_name not in anno:
            continue
        camera_extrinsic = np.array(anno[camera_name]["extrinsic"])
        if np.allclose(camera_extrinsic[:3, 3], 0, atol=thr):
            print("No abnormal translation vector.")
        else:
            print("Abnormal translation vector:", camera_extrinsic[:3, 3])

def test_update_camera_extrinsic():
    yaml_path = "/media/amdin/Drive2/lrs/Multi-V2X/Town01__2023_11_10_22_23_28/cav_217/000008.yaml"
    save_path = "/home/lrs/tmp/000008.yaml"

    update_camera_extrinsic(yaml_path, save_path)
    is_reasonable_extrinsic(save_path)

# =======================================================
# - Running
# =======================================================

if __name__ == '__main__':
    # test_update_camera_extrinsic()

    # Replace with your actual directory
    # Set "dataset_root" same as "save_root" means replace inplace.
    config = {
        "Multi-V2X": {
            "dataset_root": "/media/amdin/Drive2/lrs/Multi-V2X/",
            "save_root": "/media/amdin/Drive2/lrs/backup/anno_fix_extrinsic_250601/"
        },
        "CoRTSG": {
            "dataset_root":  "/media/amdin/Drive2/lrs/V2xTestScenario/",
            "save_root": "/media/amdin/Drive2/lrs/backup/anno_cortsg_fix_extrinsic_250601/"
        }
    }
    dataset_name = "Multi-V2X"
    dataset_root = config[dataset_name]["dataset_root"]
    save_root = config[dataset_name]["save_root"]

    if dataset_name == "Multi-V2X":
        # fix Multi-V2X
        update_camera_extrinsic_in_dataset(dataset_root, save_root)
    elif dataset_name == "CoRTSG":
        # fix CoRTSG
        update_camera_extrinsic_in_cortsg(dataset_root, save_root)
