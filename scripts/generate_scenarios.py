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

# If you get warnings about fc-list, remove font cache:
# rm -rf ~/.cache/matplotlib/

TEMP_MISSION_FILE = 'temp_mission.xml'

def rewrite_mission_file(mission_file, enable_gui):
    tree = ET.parse(mission_file)

    root = tree.getroot()

    log_dir_element = root.find('log_dir')
    log_dir_element.text = os.path.abspath(os.path.expanduser(log_dir_element.text))

    run_element = root.find('run')
    run_element.attrib['enable_gui'] = str(enable_gui)
    run_element.attrib['time_warp'] = str(enable_gui * 5 )
    run_element.attrib['start_paused'] = 'false'

    for child in root:
        if child.tag == 'seed':
            root.remove(child)

    tree.write(TEMP_MISSION_FILE)

def scale_value(OldValue, OldMin, OldMax, NewMin, NewMax):
    OldRange = (OldMax - OldMin)
    NewRange = (NewMax - NewMin)
    NewValue = (((OldValue - OldMin) * NewRange) / OldRange) + NewMin
    return NewValue

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
    return (cls(value), cls)

def expand_variable_ranges(ranges_file, num_runs, mission_dir, entity_list=None):
    if not entity_list:
        entity_count = \
            sum([1 for node in ET.parse(TEMP_MISSION_FILE).getroot()
                 if node.tag == 'entity'])
        entity_list = list(range(entity_count))

    root = ET.parse(ranges_file).getroot()

    num_of_vars = len(list(root))
    xx = pyDOE.lhs(num_of_vars*len(entity_list), samples=int(num_runs))

    # Build a list of tuples ('tag name', low, high)
    ranges_list = []
    cls_dict = {}

    for idx, entity_num in enumerate(entity_list):
        var_num = 0
        for child in root:

            try:
                high, cls = convert(child.attrib['high'], child.attrib['type'])
                low, cls = convert(child.attrib['low'], child.attrib['type'])
            except (AttributeError, KeyError):
                print('missing type or low or high in element ', child.tag)

            column_name = child.tag+ "~" + str(entity_num) + "~"

            try:
                column_name += child.attrib['attribute']
            except (AttributeError, KeyError):
                pass

            cls_dict[column_name] = cls
            ranges_list.append((column_name, high, low ))

            xx[:,var_num+idx*num_of_vars] = [cls(scale_value(x, 0.0, 1.0, low, high)) for x in xx[:,var_num+idx*num_of_vars]]
            var_num += 1

    # Build a data frame where the columns are labelled with the XML element
    # tag. Rows represent values for the variables for each run
    df = pd.DataFrame(xx, columns=[ c[0] for c in ranges_list ])
    write_scenarios(df, cls_dict, mission_dir, entity_list)

def write_scenarios(df, cls_dict, mission_dir, entity_list):

    def updater(element, variable, index):
        cls = cls_dict[variable] # get the conversion function
        val = str(cls(df.loc[index,variable]))
        node_name, ent_num, attr = variable.split("~")
        if attr:
            element.attrib[attr] = val
        else:
            element.text = val

    #print df
    variables = list(df.columns.values)
    first_vars = [v.split('~')[0] for v in variables]

    for index, row in df.iterrows():
        tree = ET.parse(TEMP_MISSION_FILE)
        root = tree.getroot()

        # Find and loop over all "entity" tags in mission file.
        entity_num = 0
        for child in root:

            try:
                idx = first_vars.index(child.tag)
                updater(child, variables[idx], index)
            except ValueError:
                pass

            if child.tag != "entity":
                continue

            if entity_list and entity_num in entity_list or entity_list == [-1]:
                for v in variables: # For every dataframe column name
                    if v != "None": # Ignore the first column
                        # Separate variable name from entity number:
                        node_name, ent_num, attr = v.split("~")
                        element = child.find(node_name) # Find the column name tag under the "entity" tag
                        if element != None and element != "None" and (int(ent_num) == entity_num or int(ent_num) == -1):
                            updater(element, v, index)
            entity_num += 1
        out_name = os.path.join(mission_dir, str(index + 1)) + '.xml'
        tree.write(out_name)


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
