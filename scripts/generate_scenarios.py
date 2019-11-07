#!/usr/bin/env python

#
# @file
#
# @section LICENSE
#
# Copyright (C) 2017 by the Georgia Tech Research Institute (GTRI)
#
# This file is part of SCRIMMAGE.
#
#   SCRIMMAGE is free software: you can redistribute it and/or modify it under
#   the terms of the GNU Lesser General Public License as published by the
#   Free Software Foundation, either version 3 of the License, or (at your
#   option) any later version.
#
#   SCRIMMAGE is distributed in the hope that it will be useful, but WITHOUT
#   ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
#   FITNESS FOR A PARTICULAR PURPOSE.  See the GNU Lesser General Public
#   License for more details.
#
#   You should have received a copy of the GNU Lesser General Public License
#   along with SCRIMMAGE.  If not, see <http://www.gnu.org/licenses/>.
#
# @author Kevin DeMarco <kevin.demarco@gtri.gatech.edu>
# @author Eric Squires <eric.squires@gtri.gatech.edu>
# @date 31 July 2017
# @version 0.1.0
# @brief Brief file description.
# @section DESCRIPTION
# A Long description goes here.
#
#

import subprocess as sp
import argparse
import xml.etree.ElementTree as ET
import os
import numpy as np
import pyDOE
import pandas as pd
import sys
import errno
import shutil
import scrimmage.utils as utils
import re
import pdb
import scrimmage.utils
import itertools
from collections import OrderedDict

# If you get warnings about fc-list, remove font cache:
# rm -rf ~/.cache/matplotlib/

TEMP_MISSION_FILE = 'temp_mission.xml'
TEMP_PARAMS_FILE = 'temp_params.xml'

def rewrite_mission_file(mission_file, enable_gui, root_log=None):
    tree = ET.parse(mission_file)

    root = tree.getroot()

    log_dir_element = root.find('log_dir')
    if root_log:
        log_dir_element.text = root_log
    else:
        log_dir_element.text = os.path.abspath(os.path.expanduser(log_dir_element.text))

    run_element = root.find('run')
    run_element.attrib['enable_gui'] = str(enable_gui)
    run_element.attrib['time_warp'] = str(enable_gui * 5 )
    run_element.attrib['start_paused'] = 'false'

    for child in root:
        if child.tag == 'seed':
            root.remove(child)

    tree.write(TEMP_MISSION_FILE)


def convert(value, type_):
    import importlib
    try:
        # Check if it's a builtin type
        if sys.version_info <= (3,0):  # python 2.x
            module = importlib.import_module('__builtin__')
        else:  # python 3
            module = importlib.import_module('builtins')
        cls = getattr(module, type_)
    except AttributeError:
        # if not, separate module and class
        module, type_ = type_.rsplit(".", 1)
        module = importlib.import_module(module)
        cls = getattr(module, type_)
    return cls(value), cls

def ranges_file_to_dict(ranges_file):
    """Convert from XML ranges file to simple dictionary"""
    root = ET.parse(ranges_file).getroot()
    ranges = OrderedDict()
    for idx, child in enumerate(root):
        ranges_dict = {}
        try:
            name = child.tag
            ranges_dict['high'], _ = convert(child.attrib['high'], child.attrib['type'])
            ranges_dict['low'], cls = convert(child.attrib['low'], child.attrib['type'])
            ranges_dict['type'] = cls
            if 'count' in child.attrib:
                ranges_dict['count'], _ = convert(child.attrib['count'], 'int')
            if 'vec' in child.attrib:
                ranges_dict['vec'], cls = convert(child.attrib['vec'], 'str')                
        except (AttributeError, KeyError):
            print('missing type or low or high in element ', child.tag)

        ranges[name] = ranges_dict

    return ranges


def expand_variable_ranges(ranges_file, num_runs, mission_dir, root_log=None, entity_list=None,
        num_repeats=1, method="lhs"):
    """Generate list of variable values to change, and associated mission files"""

    if not root_log:
        root_log = mission_dir

    ranges = ranges_file_to_dict(ranges_file)

    # Generate set of params to test
    if method == "lhs":
        # Latin-Hypercube method
        xx = np.array(pyDOE.lhs(len(ranges), samples=num_runs), dtype=object)

        for idx, (param, desc) in enumerate(ranges.items()):        
            if 'vec' in desc:
                vals = np.fromstring(desc['vec'],sep=',')
                diff = len(vals)
                xx[:, idx] = (xx[:, idx] * diff).astype(int)
                for idx2 in range(len(xx[:,idx])):               
                    xx[idx2, idx] = vals[xx[idx2,idx]]
            else:
                diff = desc['high'] - desc['low']
                xx[:, idx] = (xx[:, idx] * diff + desc['low']).astype(desc['type'])
    elif method == "grid":
        # Dumb grid search, try every combination
        val_options = []
        for param, desc in ranges.items():
            if 'count' not in desc:
                raise AttributeError("Param '{}' needs count specified for grid method"
                        .format(param))
            vals = np.linspace(desc['low'], desc['high'], desc['count']).astype(desc['type'])
            val_options.append(vals)

        # combine all lists of options to get every combination
        xx = np.array(list(itertools.product(*val_options)), dtype=object)

    # Allow multiple repetitions at each combination of parameters
    xx = np.repeat(xx, num_repeats, axis=0)

    # Build a data frame where the columns are labelled with the XML element
    # tag. Rows represent values for the variables for each run
    df = pd.DataFrame(xx, columns=ranges)

    # Add a column for run number - indexed from 1 to match everything else,
    # set this as the index
    df['run'] = [i for i in range(1, len(df) + 1)]
    df.set_index('run', inplace=True)
    # Output csv mapping params to log directories
    df.to_csv(os.path.join(root_log, 'batch_params.csv'), index=True)
    return write_scenarios(df, mission_dir)


def write_scenarios(df, mission_dir):
    # For each row in the df, replace all instances of key with the associated
    # value from LHS sampling
    mission_list = []
    mission_string = ""
    with open(TEMP_MISSION_FILE) as f:
        mission_string = f.read()
    for run, row in df.iterrows():
        this_mission = mission_string
        #  pdb.set_trace()
        for col in df:
            this_mission = replace_val(col, this_mission, row[col])
        # Output this mission file
        out_name = os.path.join(mission_dir, str(run)) + '_mission.xml'
        mission_list.append(out_name)
        with open(out_name, "w") as out_file:
            out_file.write(this_mission)

        #  print("Writing", out_name, "with params", row)
        out_params = os.path.join(mission_dir, str(run)) + '_params.xml'
        row.to_csv(out_params)

    return mission_list


def replace_val(var, mission_string, val):
    # Replace var in the xml string with the value from LHS
    reg = r"\${{{}=(.+?)}}".format(var)
    pattern = re.compile(reg)
    mission_string = pattern.sub(
            lambda m: m.group().replace(m.group(), "{}".format(str(val))),
            mission_string)
    return mission_string

def from_run_experiments(args, out_dir, mission_dir, root_log, num_repeats=1,
        method='lhs'):
    # TODO: Can we incorporate this and main together? Or should
    # generate_scenarios even be callable?
    missions = []
    rewrite_mission_file(args.mission, False, root_log)
    if args.ranges and os.path.isfile(args.ranges):
        missions += expand_variable_ranges(args.ranges, args.tasks, mission_dir, root_log,
                num_repeats=num_repeats, method=method)
    else:
        # If the user didn't supply a ranges file, just copy the temp mission
        # file and rename it for each run
        # num_repeats doesn't work here, just use tasks to control run number
        for i in range(0, args.tasks):
            fn = os.path.join(mission_dir, str(i+1), "_mission.xml")
            scrimmage.utils.make_dirs_recursive(os.path.dirname(fn))
            shutil.copyfile(TEMP_MISSION_FILE, fn)
            missions.append(fn)
    os.remove(TEMP_MISSION_FILE)

    return missions


def main():
    parser = argparse.ArgumentParser(description='Monte Carlo simulation for SCRIMMAGE.')

    add = parser.add_argument
    add('mission_file', help='input to scrimmage')
    add('out_dir', help='Directory to save generated mission files')
    add('--ranges', default="", help='ranges xml file')
    add('--entity_list', nargs='+', type=int,
        help=('list of entities for whom to alter properties ' +
              '(without this option, ranges will be applied to all entities.'
              ' use -1 to apply the same random number to each entity in a scenario')
    )
    add('--num_runs', type=int, default=1, help='number of runs')
    add('--test', action="store_true", help='show gui')
    add('--only_xml', action="store_true", help='dont run grid engine')
    add('--nodes', help="which nodes to run on")

    args = parser.parse_args()
    if not os.path.isfile(args.mission_file):
        themsg = 'Mission file ' + args.mission_file + ' not found!'
        print(themsg)
        return errno.ENOENT
    try:
        os.mkdir(args.out_dir)
    except OSError:
        pass

    rewrite_mission_file(args.mission_file, args.test)

    mission_dir = os.path.join(os.path.abspath(args.out_dir), 'missions')
    try:
        os.mkdir(mission_dir)
    except OSError:
        pass

    if os.path.isfile(args.ranges):
        expand_variable_ranges(args.ranges, args.num_runs, mission_dir, args.entity_list)
    else:
        # If the user didn't supply a ranges file, just copy the temp mission
        # file and rename it for each run
        for i in range(0,args.num_runs):
            shutil.copyfile(TEMP_MISSION_FILE, mission_dir+"/"+str(i+1)+".xml")

    if not args.only_xml:
        utils.qsub(args.num_runs, mission_dir, args.nodes)

    os.remove(TEMP_MISSION_FILE)
    return 0

if __name__ == '__main__':
    sys.exit(main())
