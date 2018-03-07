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
          $0 <PluginType> <PluginName> <ProjectDirectory>

OPTIONS:
          -h Display this usage information

NOTES:
          PluginType must be one of the following:
          autonomy, motion, interaction, controller, metrics, sensor, network
EOF
}


if [ "$#" -lt 3 ]; then
    usage
    exit 2
fi

PLUGIN_TYPE="$1"
PLUGIN_NAME="$2"
PROJECT_DIR="$(readlink -f $3)"

if [ $PLUGIN_TYPE != "autonomy" ] && [ $PLUGIN_TYPE != "motion" ]  && \
   [ $PLUGIN_TYPE != "interaction" ] && [ $PLUGIN_TYPE != "controller" ] && \
   [ $PLUGIN_TYPE != "metrics" ] && [ $PLUGIN_TYPE != "sensor" ] && \
   [ $PLUGIN_TYPE != "network" ]; then
    echo "Invalid plugin type."
    echo "PluginType must be \"autonomy\", \"motion\", \"controller\", "
    echo "\"metrics\", \"sensor\", \"network\", or \"interaction\""
    exit 2
fi

if [ ! -d "$PROJECT_DIR" ];
then
    echo "ProjectDirectory doesn't exist."
    exit 1
fi
PROJECT_DIR=$(readlink -f "${PROJECT_DIR}")
PROJECT_NAME=$(basename "${PROJECT_DIR}")

HEADER_GUARD="INCLUDE_${PROJECT_NAME}_PLUGINS_${PLUGIN_TYPE}_${PLUGIN_NAME}_${PLUGIN_NAME}_H_"
HEADER_GUARD=$(echo $HEADER_GUARD | awk '{print toupper($0)}') # to uppercase
HEADER_GUARD=$(echo $HEADER_GUARD | tr - _) # replace - with _

# Jump to the directory that holds this script
DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
pushd ${DIR} >& /dev/null

###############################################################################
# Use the plugin template to generate the plugin
###############################################################################
H_FILE_IN=$(readlink -f ./templates/$PLUGIN_TYPE/PluginTemplate.h)
CPP_FILE_IN=$(readlink -f ./templates/$PLUGIN_TYPE/PluginTemplate.cpp)
XML_FILE_IN=$(readlink -f ./templates/$PLUGIN_TYPE/PluginTemplate.xml)
CMAKE_FILE_IN=$(readlink -f ./templates/$PLUGIN_TYPE/CMakeLists.txt)

OUT_SRC_DIR="${PROJECT_DIR}/src/plugins/${PLUGIN_TYPE}/${PLUGIN_NAME}"
OUT_H_DIR="${PROJECT_DIR}/include/${PROJECT_NAME}/plugins/${PLUGIN_TYPE}/${PLUGIN_NAME}"

mkdir -p ${OUT_SRC_DIR}
mkdir -p ${OUT_H_DIR}

H_FILE_OUT="${OUT_H_DIR}/$PLUGIN_NAME.h"
CPP_FILE_OUT="${OUT_SRC_DIR}/$PLUGIN_NAME.cpp"
XML_FILE_OUT="${OUT_H_DIR}/$PLUGIN_NAME.xml"
CMAKE_FILE_OUT="${OUT_SRC_DIR}/CMakeLists.txt"

cp $H_FILE_IN $H_FILE_OUT
cp $CPP_FILE_IN $CPP_FILE_OUT
cp $XML_FILE_IN $XML_FILE_OUT
cp $CMAKE_FILE_IN $CMAKE_FILE_OUT

sed -i -- "s/(>>>PLUGIN_NAME<<<)/$PLUGIN_NAME/g" $H_FILE_OUT
sed -i -- "s/(>>>HEADER_GUARD<<<)/$HEADER_GUARD/g" $H_FILE_OUT
sed -i -- "s/(>>>PLUGIN_NAME<<<)/$PLUGIN_NAME/g" $XML_FILE_OUT
sed -i -- "s/(>>>PLUGIN_NAME<<<)/$PLUGIN_NAME/g" $CMAKE_FILE_OUT

sed -i -- "s/(>>>PLUGIN_NAME<<<)/$PLUGIN_NAME/g" $CPP_FILE_OUT
sed -i -- "s/(>>>PROJECT_NAME<<<)/$PROJECT_NAME/g" $CPP_FILE_OUT

pushd >& /dev/null
