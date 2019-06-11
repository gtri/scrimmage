#!/usr/bin/env python

import os

def env(args):
    print('SCRIMMAGE_PLUGIN_PATH=' + os.environ['SCRIMMAGE_PLUGIN_PATH'])
    print('SCRIMMAGE_MISSION_PATH=' + os.environ['SCRIMMAGE_MISSION_PATH'])
    print('SCRIMMAGE_DATA_PATH=' + os.environ['SCRIMMAGE_DATA_PATH'])
    print('SCRIMMAGE_CONFIG_PATH=' + os.environ['SCRIMMAGE_CONFIG_PATH'])
