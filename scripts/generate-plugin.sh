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

if [ "$#" -lt 3 ]; then
    echo "usage: $0 <PluginType> <PluginName> <ProjectDirectory>"
    exit 2
fi

PLUGIN_TYPE="$1"
PLUGIN_NAME="$2"
PROJECT_DIR="$(readlink -f $3)"

if [ $PLUGIN_TYPE != "autonomy" ] && [ $PLUGIN_TYPE != "motion" ]  && \
   [ $PLUGIN_TYPE != "interaction" ] && [ $PLUGIN_TYPE != "controller" ] && \
   [ $PLUGIN_TYPE != "metrics" ] && [ $PLUGIN_TYPE != "sensor" ]; then
    echo "Invalid plugin type."
    echo "PluginType must be \"autonomy\", \"motion\", \"controller\", "
    echo "\"metrics\", \"sensor\", or \"interaction\""
    exit 2
fi

if [ ! -d "$PROJECT_DIR" ];
then
    echo "ProjectDirectory doesn't exist."
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
sed -i -- "s/(>>>PLUGIN_NAME<<<)/$PLUGIN_NAME/g" $XML_FILE_OUT
sed -i -- "s/(>>>PLUGIN_NAME<<<)/$PLUGIN_NAME/g" $CMAKE_FILE_OUT

sed -i -- "s/(>>>PLUGIN_NAME<<<)/$PLUGIN_NAME/g" $CPP_FILE_OUT
sed -i -- "s/(>>>PROJECT_NAME<<<)/$PROJECT_NAME/g" $CPP_FILE_OUT

pushd >& /dev/null

################################################################################
## Modify the CMakeList.txt file in the plugin directory
################################################################################
# This is no longer needed. There is a CMake script that GLOBS the
#subdirectories of the plugins directory.  

#CMAKELISTS_FILE=$(readlink -f "$PROJECT_DIR/CMakeLists.txt")
#
#if [ ! -e $CMAKE_FILE ];
#then
#    echo "Can't edit plugins CMakeLists.txt file, it doesn't exist."
#    exit -1
#fi
#
## We need to add the "add_subdirectory(MyPlugin)" line to the CMakeLists.txt
## file. First, we need to see if the line already exists. If it does exist,
## exit successfully, if not, we need to add the line.
#LINE_VALUE="add_subdirectory(${PLUGIN_NAME})"
#
## See if the CMakeLists.txt file already contains the line:
#grep -i -q $LINE_VALUE $CMAKELISTS_FILE
#LINE_EXISTS=$?
#
## grep returns 0 if the line exists.
#if [ $LINE_EXISTS -eq 0 ];
#then
#    # We don't have to edit file
#    exit 0
#fi
#
## The line doesn't exist yet, append it to the CMakeLists.txt file
#echo $LINE_VALUE >> $CMAKELISTS_FILE
