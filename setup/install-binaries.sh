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
usage: sudo $0
This script installs all required dependencies.

OPTIONS

    --external
        if --external is passed, only install what is necessary for an external build
        (see EXTERNAL flag to project CMakeLists.txt). Otherwise do full
        installation.
EOF
}

###################################################################
# Dependencies array.
# Add new dependencies here.
###################################################################
# Dependencies only for Ubuntu
UBUNTU_VERSION=$(cat /etc/lsb-release | grep DISTRIB_RELEASE | cut -d '=' -f 2)

DEPS_DPKG=(
    sudo
    git
    cmake
    gcc
    build-essential
    librapidxml-dev
    libxml2-dev
    libxmlsec1-dev
    libeigen3-dev
    libgeographic-dev
    libboost-thread-dev
    libboost-date-time-dev
    libboost-graph-dev
    libboost-iostreams-dev
    libboost-program-options-dev
    libboost-regex-dev
    libboost-filesystem-dev
    libboost-system-dev
    autoconf
    automake
    libtool
    curl
    unzip
)

if [ "$EXTERNAL" = false ]; then
    DEPS_DPKG+=(
        ccache
        parallel
        libbullet-dev
        graphviz
        doxygen
        libopencv-dev
    )
fi

if [ "20.04" == ${UBUNTU_VERSION} ]; then
    DEPS_DPKG+=(
    libvtk6.3-qt
    libgrpc-dev
    libgrpc++-dev
    ) #GRPC Libraries for Focal
fi
if [ "22.04" == ${UBUNTU_VERSION} ]; then
    echo "Detected Ubuntu 22"
    DEPS_DPKG+=(
    libvtk9-qt-dev
    libgrpc-dev
    libgrpc++-dev
    tcl-vtk7
    libvtk9-dev
    protobuf-compiler
    protobuf-compiler-grpc
    pybind11-dev
    libprotobuf-dev
    ) #GRPC Libraries for Jammy
fi

#Require the script to be run as root
if [[ $(/usr/bin/id -u) -ne 0 ]]; then
    echo "This script must be run as root because libraries will be installed."
    usage
    exit
fi

###################################################################
# Determine which package system is being used
# This helps determine which version of linux is being used
###################################################################
# Ubuntu
if which apt-get &> /dev/null; then
    echo "This is Ubuntu. Using dpkg."

    # OpenSuse, Mandriva, Fedora, CentOs, ecc. (with rpm)
elif which rpm &> /dev/null; then
    DEPENDENCIES=("${DEPS_COMMON[@]}" "${DEPS_RPM[@]}")
    echo "This is Red Hat / CentOS. Using rpm."

    # ArchLinux (with pacman)
elif which pacman &> /dev/null; then
    DEPENDENCIES=("${DEPS_COMMON[@]}" "${DEPS_PACMAN}")
    echo "This is ArchLinux. Using pacman."
else
    echo "Can't determine operating system or package system."
    exit
fi

###################################################################
# Determine which packages are missing
###################################################################
echo "Detecting which required packages are not installed."

dep_len=${#DEPENDENCIES[@]}

PKGSTOINSTALL=""
for (( i=0; i < $dep_len; i++))
do
    if which apt-get &> /dev/null; then
	    # some packages have a ':' so check for that too
	    if [[ ! `dpkg -l | grep -w "ii  ${DEPENDENCIES[$i]} "` ]] &&
	           [[ ! `dpkg -l | grep "ii  ${DEPENDENCIES[$i]}:"` ]]; then
	        PKGSTOINSTALL=$PKGSTOINSTALL" "${DEPENDENCIES[$i]}
	    fi
	    # OpenSuse, Mandriva, Fedora, CentOs, ecc. (with rpm)
    elif which rpm &> /dev/null; then
	    if [[ `rpm -q ${DEPENDENCIES[$i]}` == "package "${DEPENDENCIES[$i]}" is not installed" ]]; then
	        PKGSTOINSTALL=$PKGSTOINSTALL" "${DEPENDENCIES[$i]}
	    fi
	    # ArchLinux (with pacman)
    elif which pacman &> /dev/null; then
	    if [[ ! `pacman -Qqe | grep "${DEPENDENCIES[$i]}"` ]]; then
	        PKGSTOINSTALL=$PKGSTOINSTALL" "${DEPENDENCIES[$i]}
	    fi
    else
	    # If it's impossible to determine if there are missing dependencies, mark all as missing
	    PKGSTOINSTALL=$PKGSTOINSTALL" "${DEPENDENCIES[$i]}
    fi
done

###################################################################
# Install missing dependencies.
# First, ask user.
###################################################################
if [ "$PKGSTOINSTALL" != "" ]; then
    #echo "The following dependencies are missing:"
    #echo "${PKGSTOINSTALL}"
    #echo -n "Want to install them? (Y/n): "
    #read SURE
    SURE="Y"
    # If user want to install missing dependencies
    if [[ $SURE = "Y" || $SURE = "y" || $SURE = "" ]]; then
        # Debian, Ubuntu and derivatives (with apt-get)
	    if which apt-get &> /dev/null; then
            apt-get update
	        apt-get install -y $PKGSTOINSTALL
	        # OpenSuse (with zypper)
	    elif which zypper &> /dev/null; then
	        zypper in $PKGSTOINSTALL
	        # Mandriva (with urpmi)
	    elif which urpmi &> /dev/null; then
	        urpmi $PKGSTOINSTALL
	        # Fedora and CentOS (with yum)
	    elif which yum &> /dev/null; then
	        echo "yum install $PKGSTOINSTALL"
	        yum install $PKGSTOINSTALL
	        # ArchLinux (with pacman)
	    elif which pacman &> /dev/null; then
	        pacman -Sy $PKGSTOINSTALL
	        # Else, if no package manager has been found
	    else
	        # Set $NOPKGMANAGER
	        NOPKGMANAGER=TRUE
	        echo "ERROR: impossible to find a package manager in your system. Please install the following packages manually: ${DEPENDENCIES[*]}."
	    fi

        # Check if installation is successful
	    if [[ $? -eq 0 && ! $NOPKGMANAGER == "TRUE" ]] ; then
	        echo "All dependencies are satisfied."
        else
	        # Else, if installation isn't successful
	        echo "ERROR: impossible to install some missing dependencies. Please install the following packages manually: ${DEPENDENCIES[*]}."
	    fi

    else
	    # Else, if user don't want to install missing dependencies
	    echo "WARNING: Some dependencies may be missing. Please install the following packages manually: ${DEPENDENCIES[*]}."
    fi
else
    echo "All dependencies are installed. No further action is required."
fi
