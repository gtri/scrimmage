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
# @author Gregory Cooke <gregory.cooke@gtri.gatech.edu>
# @date 28 November 2018
# @version 0.1.0
# @brief Brief file description.
# @section DESCRIPTION
# A Long description goes here.
#
#

from __future__ import (absolute_import, division,
                        print_function, unicode_literals)
try:
    from builtins import *
except ImportError:
    from __builtin__ import *
    

import os
import sys
import errno
import multiprocessing
import subprocess
import datetime
import argparse
import tqdm
import generate_scenarios
import scrimmage.utils


def call_scrimmage(arg):
    subprocess.call(arg, shell=True)

def symlink_force(target, link_name):
    """Create symlink even if one already exists"""
    try:
        os.symlink(target, link_name)
    except OSError as e:
        if e.errno == errno.EEXIST:
            os.remove(link_name)
            os.symlink(target, link_name)
        else:
            raise e

parser = argparse.ArgumentParser(
        description="Run batch SCRIMMAGE experiments",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter
        )
parser.add_argument('-m', '--mission', help='Mission file',
                    default= "./scrimmage/scrimmage/missions/batch-example-mission.xml")
parser.add_argument('-r', '--ranges', help='Ranges file for generating new'
                    ' scenarios', default= "~/scrimmage/scrimmage/config/ranges/batch-ranges.xml")
parser.add_argument('-p', '--parallel', help='Maximum number of parallel runs',
                    default=7, type=int)
parser.add_argument('-n', '--experiment-name', help='Name of the batch '
                    'experiment',
                    default=datetime.datetime.now().strftime(
                        "experiment_%Y-%m-%d_%H-%M-%S"))
parser.add_argument('-l', '--log-dir', help='Log directory',
                    default="~/.scrimmage/experiments")
parser.add_argument('-s', '--num-repeats', help='Num repeats per set of params',
                    default=1, type=int)
parser.add_argument('-d', '--method', help='Method of sampling, grid or lhs',
                    default='lhs', type=str)
parser.add_argument('-t', '--tasks', help='Number of simulation runs',
                    default=100, type=int)

args = parser.parse_args()

if not args.mission:
    sys.exit('No mission file specified')

if not os.path.isfile(args.mission):
    sys.exit('Mission file does not exist')

# See if the user is using the ranges file
if not args.ranges:
    print('not using ranges file')
elif not os.path.exists(os.path.expanduser(args.ranges)):
    sys.exit('Ranges file does not exist... Exiting')

if args.method == "lhs":
    if args.tasks is None:
        parser.error("Cannot use lhs without -t (--tasks) specifying how many runs")
elif args.method == "grid":
    if args.tasks is not None:
        print("Warning, --tasks ignored, using counts in ranges file")

print("Using parameter selection method: {}".format(args.method))

MISSION_FILE = args.mission

EXPERIMENTS_DIR = os.path.expanduser(args.log_dir)
ROOT_LOG = os.path.join(EXPERIMENTS_DIR, args.experiment_name)

# Add latest symlink to experiments folder
scrimmage.utils.make_dirs_recursive(EXPERIMENTS_DIR)
symlink_force(ROOT_LOG, os.path.join(EXPERIMENTS_DIR, "latest"))

if os.path.exists(ROOT_LOG):
    sys.exit("Experiment name already exists")

LOG_FILE_DIR = os.path.join(ROOT_LOG, "logs")
MISSION_FILE_DIR = os.path.join(ROOT_LOG, "missions")

if not os.path.exists(LOG_FILE_DIR):
    os.makedirs(LOG_FILE_DIR)
if not os.path.exists(MISSION_FILE_DIR):
    os.makedirs(MISSION_FILE_DIR)

# generates set of mission files
ids_files = generate_scenarios.from_run_experiments(
                args, LOG_FILE_DIR, MISSION_FILE_DIR,
                ROOT_LOG, num_repeats=args.num_repeats,
                method=args.method)

STARTTIME = datetime.datetime.now()

# Make the pool of processors
pool = multiprocessing.Pool(args.parallel)
# TODO: Kevin fix logging. This script doesn't know about the folders that
# scrimmage generates
tasks = ['scrimmage -j 0 -t {0} {1} > {2}/{0}.log 2>&1'.format(i + 1,
         id_file, LOG_FILE_DIR) for i, id_file in enumerate(ids_files)]
# Map tasks to pools
results = []
imap = pool.imap_unordered(call_scrimmage, tasks)
# tqdm lets us get a progress bar for long experiments
for res in tqdm.tqdm(imap, total=len(tasks)):
    results.append(res)
# Don't let anything else go to the pool and wait for everything in the pool to
# finish
pool.close()
pool.join()

ENDTIME = datetime.datetime.now()
print("Completed {} simulations in {} seconds".format(len(tasks), (ENDTIME -
      STARTTIME).total_seconds()))
