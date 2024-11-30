#!/usr/bin/env python
# -*- coding: utf-8 -*-
'''
@File    :   logging_utils.py
@Date    :   2024-01-20
@Author  :   Rongsong Li <rongsong.li@qq.com>
@Version :   1.0
@Desc    :   Logging process info
'''

import os
import sys
import logging

def init_logger(name, save_path=None, level=logging.INFO):
    """
    Initialize the logging.

    Parameters
    ----------
    name : str
        Logger's name

    save_path: str
        The log file will be saved in

    level : str
        

    Returns
    -------
    logger:
        The instance for logging.
    
    """
    
    logger = logging.getLogger(name)

    # the logger have been initialized
    if  logger.handlers:
        return logger
    
    logger.setLevel(level)

    # formatter
    ffmt = logging.Formatter(
        fmt = "[%(asctime)s][%(levelname)s][%(filename)s:%(lineno)d] %(message)s",
        datefmt = "%Y-%m-%d %H:%M:%S"
    )
    sfmt = logging.Formatter(
        fmt = "[%(levelname)s][%(filename)s:%(lineno)d] %(message)s"
    )

    # file handler
    if save_path:
        fh = logging.FileHandler(filename=save_path, encoding="utf8")
        fh.setLevel(level)
        fh.setFormatter(ffmt)
        logger.addHandler(fh)

    # stream handler
    sh = logging.StreamHandler()
    sh.setLevel(level = level)
    sh.setFormatter(sfmt)
    # when using CARLA, stream handler is redundant
    # logger.addHandler(sh)

    return logger