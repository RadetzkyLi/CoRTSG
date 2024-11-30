#!/usr/bin/env python
# -*- coding: utf-8 -*-
'''
@File    :   setup.py
@Date    :   2024-05-24
@Author  :   Rongsong Li <rongsong.li@qq.com>
@Version :   1.0
@Desc    :   Risky Testing Scenario Generation for Cooperative Perception
'''


from os.path import dirname, realpath
from setuptools import setup, find_packages, Distribution


def _read_requirements_file():
    """Return the elements in requirements.txt."""
    req_file_path = '%s/requirements.txt' % dirname(realpath(__file__))
    with open(req_file_path) as f:
        return [line.strip() for line in f]


setup(
    name='CoRTSG',
    version="1.0",
    packages=find_packages(),
    url='',
    license='MIT',
    author='Rongsong Li',
    author_email='rongsong.li@qq.com',
    description='A general and effective framework generating risky testing scenarios for cooperative perception',
    long_description=open("README.md").read(),
    install_requires=_read_requirements_file(),
)
