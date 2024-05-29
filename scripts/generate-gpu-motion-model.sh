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

usage()
{
    cat << EOF

USAGE:
          $0 <Kernel Name> <KernelDirectory>

OPTIONS:
          -h Display this usage information

NOTES:
EOF
}


if [ "$#" -lt 2 ]; then
    usage
    exit 2
fi

KERNEL_NAME="$1"
PROJECT_DIR="$(readlink -f $2)"

if [ ! -d "$PROJECT_DIR" ];
then
    echo "ProjectDirectory ${PROJECT_DIR} doesn't exist."
    exit 1
fi
PROJECT_DIR=$(readlink -f "${PROJECT_DIR}")
PROJECT_NAME=$(basename "${PROJECT_DIR}")

# Jump to the directory that holds this script
DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
pushd ${DIR} >& /dev/null

###############################################################################
# Use the plugin template to generate the plugin
###############################################################################
FILE_IN=$(readlink -f ./templates/gpu_motion/GPU_Motion_Model_Template.cl)

OUT_DIR="${PROJECT_DIR}/${KERNEL_NAME}/"

mkdir -p ${OUT_DIR}

FILE_OUT="${OUT_DIR}/${KERNEL_NAME}.cl"

cp ${FILE_IN} ${FILE_OUT}

KERNEL_NAME_SNAKE=$(echo ${KERNEL_NAME} | sed -E 's/([^[:upper:]_])([[:upper:]])/\1_\2/g')
KERNEL_NAME_LOWER_SNAKE=$(echo ${KERNEL_NAME_SNAKE} | sed -E 's/[[:upper:]]/\L&/g')
KERNEL_NAME_UPPER_SNAKE=$(echo ${KERNEL_NAME_SNAKE} | sed -E 's/[[:alpha:]]/\U&/g')

sed -i -- "s/(>>>KERNEL_NAME<<<)/$KERNEL_NAME/g" $FILE_OUT
sed -i -- "s/(>>>KERNEL_NAME_LOWER_SNAKE<<<)/$KERNEL_NAME_LOWER_SNAKE/g" $FILE_OUT
sed -i -- "s/(>>>KERNEL_NAME_UPPER_SNAKE<<<)/$KERNEL_NAME_UPPER_SNAKE/g" $FILE_OUT

pushd >& /dev/null
