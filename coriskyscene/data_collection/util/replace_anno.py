#!/usr/bin/env python
# -*- coding: utf-8 -*-
'''
@File    :   replace_anno.py
@Date    :   2025-05-31
@Author  :   Rongsong Li <rongsong.li@qq.com>
@Version :   1.0
@Desc    :   Replace annotations with new ones.
'''

import os
import glob
import shutil
from tqdm import tqdm
import numpy as np

from joblib import Parallel,delayed

# =======================================================
# - Backup 
# =======================================================
def backup_anno_for_agent(agent_root, backup_root):
    pattern = os.path.join(agent_root, "*.yaml")
    paths = glob.glob(pattern)
    paths = [path for path in paths if not path.endswith("_view.yaml")]
    agent = os.path.basename(agent_root)
    scenario_name = os.path.basename(os.path.dirname(agent_root))
    save_dir = os.path.join(backup_root, scenario_name, agent)
    if os.path.exists(save_dir): # dir should be empty
        return
    else:
        os.makedirs(save_dir)
    for path in paths:
        shutil.copy2(path, os.path.join(save_dir, os.path.basename(path)))

def backup_anno_for_dataset(dataset_root, backup_root):
    """Backup the annotations"""
    for scenario_name in tqdm(os.listdir(dataset_root)):
        scenario_root = os.path.join(dataset_root, scenario_name)
        agents = [agent for agent in os.listdir(scenario_root) if agent.startswith("cav") or
                agent.startswith("rsu")]
        agents.append("map")
        Parallel(n_jobs=8, backend="multiprocessing")(delayed(backup_anno_for_agent)
                                                      (os.path.join(scenario_root, agent), backup_root)
                                                      for agent in agents)
        
def backup_anno_for_scenario(scenario_root:str, cur_backup_root:str):
    pattern = os.path.join(scenario_root, "*/*.yaml")
    paths = glob.glob(pattern)
    paths = [path for path in paths if not path.endswith("_view.yaml")]
    scenario_name = os.path.basename(scenario_root)
    for path in paths:
        agent = os.path.basename(os.path.dirname(path))
        save_dir = os.path.join(cur_backup_root, scenario_name, agent)
        os.makedirs(save_dir, exist_ok=True)
        shutil.copy2(path, os.path.join(save_dir, os.path.basename(path)))

def backup_anno_for_cortsg(dataset_root:str, backup_root:str):
    for logical_scenario in os.listdir(dataset_root):
       print("Logical scenario:", logical_scenario)
       logical_scenario_root = os.path.join(dataset_root, logical_scenario)
       cur_backup_root = os.path.join(backup_root, logical_scenario)
       concrete_scenario_roots = [os.path.join(logical_scenario_root, x) 
                                  for x in os.listdir(logical_scenario_root)]
       Parallel(n_jobs=8, backend="multiprocessing")(delayed(backup_anno_for_scenario)
                                                    (root, cur_backup_root) for root in concrete_scenario_roots)
       

# =======================================================
# - Replace
# =======================================================
def replace_anno(anno_path:str, new_anno_path:str):
    if not os.path.exists(new_anno_path):
        print("Warning: New annotation file not exist:", new_anno_path)
        return 0
    shutil.copy2(new_anno_path, anno_path)
    return 1

def replace_anno_for_dataset(dataset_root:dir, new_anno_root:dir):
    """Replace annotaion files (yaml) in dataset root with the new ones.

    Notes
    -----
    1. Data in `dataset_root` and `new_anno_root` should share the same structure!!!
    """
    cnt_total = 0
    for scenario_name in tqdm(os.listdir(dataset_root)):
        scenario_dir = os.path.join(dataset_root, scenario_name)
        cnt = 0
        for agent_name in os.listdir(scenario_dir):
            if not (agent_name.startswith("cav") or
                    agent_name.startswith("rsu")):
                continue
            pattern = os.path.join(scenario_dir, agent_name, "*.yaml")
            yaml_paths = glob.glob(pattern)
            yaml_paths = [x for x in yaml_paths if not x.endswith("_view.yaml")]

            # replace
            new_anno_agent_root = os.path.join(new_anno_root, scenario_name, agent_name)
            new_anno_agent_paths = [os.path.join(new_anno_agent_root, os.path.basename(x)) for x in yaml_paths]
            res = Parallel(n_jobs=16, backend="multiprocessing", verbose=0)(
                delayed(replace_anno)(x, y)
                for x,y in zip(yaml_paths, new_anno_agent_paths)
            )
            cnt += np.sum(res)

        print("{0} {1} done! {2} yamls were replaced!{0}".format("*"*10, scenario_name, cnt))
        cnt_total += cnt

    print("{0} yaml files were replaced!".format(cnt_total))


def replace_anno_in_scenario(scenario_root:str, new_anno_scenario_root:str):
    """Work for CoRTSG bacause few timestamps in a scenario"""
    cnt = 0
    for agent_name in os.listdir(scenario_root):
        if not (agent_name.startswith("cav") or
                agent_name.startswith("rsu")):
            continue
        pattern = os.path.join(scenario_root, agent_name, "*.yaml")
        yaml_paths = glob.glob(pattern)
        yaml_paths = [x for x in yaml_paths if not x.endswith("_view.yaml")]

        # replace
        for dest_path in yaml_paths:
            fname = os.path.basename(dest_path)
            src_path = os.path.join(new_anno_scenario_root, agent_name, fname)
            if not os.path.exists(src_path):
                print("Warning: New annotation file not exist:", src_path)
                continue
            shutil.copy2(src_path, dest_path)
            cnt += 1

    return cnt

def replace_anno_for_cortsg(dataset_root:str, new_anno_scenario_root:str):
    cnt_total = 0
    for logical_scenario in os.listdir(dataset_root):
       if logical_scenario in ['parkingexit', 'example', 'juncreverse', 'juncreverse_v2v',
                                'juncrush', 'juncsame', 'juncsame_v2v', 'juncturn', 'juncturn_v2v']:
           continue
       
       logical_scenario_root = os.path.join(dataset_root, logical_scenario)
       concrete_scenario_roots = [os.path.join(logical_scenario_root, x) 
                                  for x in os.listdir(logical_scenario_root)]
       new_anno_concrete_scenario_roots = [os.path.join(new_anno_scenario_root, logical_scenario, x)
                                       for x in os.listdir(logical_scenario_root)]
       res = Parallel(n_jobs=16, backend="loky", verbose=1)(delayed(replace_anno_in_scenario)
                                                        (x, y) for x,y in 
                                                        zip(concrete_scenario_roots,
                                                            new_anno_concrete_scenario_roots))
       
       cnt = np.sum(res)
       print("{0} {1} done! {2} yamls were replaced{0}".format("*"*10, logical_scenario, cnt))
       cnt_total += cnt
    print("{0} yaml files were replaced!".format(cnt_total))
        
# =======================================================
# - Test
# =======================================================
def test_replace_anno():
    pass

# =======================================================
# - Running
# =======================================================


if __name__ == '__main__':
    # This process may take hours depending on 
    # performance of your disk.
    # Replace with your actual directory.
    config = {
        "Multi-V2X": {
            "dataset_root": "/media/amdin/Drive2/lrs/Multi-V2X/",
            "new_anno_root": "/media/amdin/Drive2/lrs/backup/anno_fix_extrinsic_250601/"
        },
        "CoRTSG": {
            "dataset_root": "/media/amdin/Drive2/lrs/V2xTestScenario/",
            "new_anno_root": "/media/amdin/Drive2/lrs/backup/anno_cortsg_fix_extrinsic_250601/"
        }
    }
   
    dataset_name = "CoRTSG"
    dataset_root = config[dataset_name]["dataset_root"]
    new_anno_root = config[dataset_name]["new_anno_root"]

    if dataset_name == "Multi-V2X":
        # fix Multi-V2X
        replace_anno_for_dataset(dataset_root, new_anno_root)
    elif dataset_name == "CoRTSG":
        # fix CoRTSG
        replace_anno_for_cortsg(dataset_root, new_anno_root)
