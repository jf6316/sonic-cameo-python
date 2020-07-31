#!/usr/bin/env python

import os
from setuptools import setup
os.listdir

setup(
       name='esc601-32q',
       version='1.0.0',
       description='Module to initialize Cameo Esc601-32q platforms',

       packages=['esc601-32q'],
       package_dir={'esc601-32q': 'esc601-32q/classes'},
    )
