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

function write_source_line {
    if [ -e "$1" ]; then
        SOURCE_LINE="source $(readlink -f $2)"
        grep "$SOURCE_LINE" "$1" > /dev/null 2>&1
        if [ $? != 0 ]; then
            echo $SOURCE_LINE >> $1
        fi
    fi
}

# The output directory is the first argument
SCRIMMAGE_LOCAL_CONFIG_DIR=$(readlink -f "$1")
PROJECT_NAME="$2"

touch ${SCRIMMAGE_LOCAL_CONFIG_DIR}/setup.bash
write_source_line "${SCRIMMAGE_LOCAL_CONFIG_DIR}/setup.bash" "${SCRIMMAGE_LOCAL_CONFIG_DIR}/env/${PROJECT_NAME}-setenv"
