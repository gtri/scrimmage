#!/bin/bash

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

usage()
{
    cat << EOF

USAGE: 
          $0 [OPTIONS]

OPTIONS:
          -h   Display this usage information
          -t   Sets the number of simulation runs (tasks)
          -m   Mission file
          -r   Ranges file for generating new scenarios
          -p   Maximum number of parallel runs (tasks)

DEFAULTS:
          -t=2   # 2 simulation runs
          -p=1   # 1 simulation at-a-time

EXAMPLE:
          Run 10 simulations, 4 at-a-time, straight.xml mission, 
          with test-1.xml ranges file:
              $0 -t 10 -p 4 -m ~/scrimmage/scrimmage/missions/straight.xml \\
              -r ~/scrimmage/scrimmage/config/ranges/test-1.xml
EOF
}

TASKS=2
MAX_PARALLEL_TASKS=1
MISSION_FILE="UNDEFINED"
RANGES_FILE="UNDEFINED"
while getopts ":t:m:r:p:h" opt; do
    case $opt in
        h)
            usage
            exit 0
            ;;
        t)
            TASKS=$OPTARG
            ;;
        m)
            MISSION_FILE=$OPTARG
            ;;
        r)
            RANGES_FILE=$OPTARG
            ;;
        p)
            MAX_PARALLEL_TASKS=$OPTARG
            ;;
        \?)
            echo "Invalid option: -$OPTARG" >&2
            usage
            exit 1
            ;;
        :)
            echo "Option -$OPTARG requires an argument." >&2
            usage
            exit 1
            ;;
    esac
done

if [ ! -e $MISSION_FILE ]; then
    echo "Mission file doesn't exist: $MISSION_FILE"
    usage
    exit -1
fi

# see if the user is using the ranges file
RANGES_OPTION=""
if [ -e $RANGES_FILE ]; then
    RANGES_OPTION="--ranges=$(readlink -f $RANGES_FILE)"
fi

if [ $MAX_PARALLEL_TASKS -gt $(nproc) ]; then
    echo "===================================================================="
    echo "WARNING: MAX_PARALLEL_TASKS is set to $MAX_PARALLEL_TASKS,"
    echo "but you only have $(nproc) processors."
    echo "===================================================================="
fi

MISSION_FILE=$(readlink -f $MISSION_FILE)

ROOT_LOG=$(readlink -f "${HOME}/swarm-log")
LOG_FILE_DIR=${ROOT_LOG}/log
MISSION_FILE_DIR=${ROOT_LOG}/missions

mkdir -p $LOG_FILE_DIR
mkdir -p $ROOT_LOG

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
pushd $SCRIPT_DIR >& /dev/null

python generate_scenarios.py \
    --num_runs=$TASKS ${RANGES_OPTION} \
    --only_xml \
    $MISSION_FILE \
    $ROOT_LOG

GENERATE_STATUS=$?
if [ ! $GENERATE_STATUS -eq 0 ]; then
    echo "Failed to generate scenarios"
    usage
    exit -1
fi

# Load up an array with mission IDs and mission files.
IDS_FILES=()
for i in `seq 1 ${TASKS}`;
do
    MISSION=$(readlink -f "${MISSION_FILE_DIR}/${i}.xml")
    IDS_FILES=("${IDS_FILES[@]}" "$i" "${MISSION}")
done

STARTTIME=$(date +%s)
parallel --no-notice --bar -j $MAX_PARALLEL_TASKS -n 2 \
    "scrimmage -j 0 -t {1} {2} > ${LOG_FILE_DIR}/{1}.log 2>&1" \
    ::: ${IDS_FILES[@]}
ENDTIME=$(date +%s)

echo "Completed ${TASKS} simulations in $(($ENDTIME - $STARTTIME)) seconds"

popd >& /dev/null
