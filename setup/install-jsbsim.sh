#!/bin/bash

SCRIMMAGE_ROOT=/opt/scrimmage
JSBSIM_ROOT=${SCRIMMAGE_ROOT}/etc/jsbsim
JSBSIM_TMP=${JSBSIM_ROOT}/tmp

JSBSIM_VERSION=1.2.0
JSBSIM_RELEASE_URL=https://github.com/JSBSim-Team/jsbsim/releases/download/v${JSBSIM_VERSION}
JSBSIM_DEVEL=JSBSIM-devel_1.2.0-1191.jammy.amd64.deb
JSBSIM_EXEC=JSBSIM_1.2.0-1191.jammy.amd64.deb

declare -A DEPS
DEPS[jsbsim-devel]=${JSBSIM_DEVEL}
DEPS[jsbsim]=${JSBSIM_EXEC}

which apt-get &> /dev/null

# Create tmp dir. If it exists, error so we don't overwrite anything
if [ ! -d "${JSBSIM_TMP}" ];
then
    mkdir -p ${JSBSIM_TMP} || echo "Failed to create ${JSBSIM_TMP}"
else
    echo "Temporary directory ${JSBSIM_TMP} already exists. Please remove 
    to proceed with installation of JSBSIM" 
    exit 1
fi

# Install jsbsim & jsbsim-devl from deb packages
for JSBSIM_DEP in "${!DEPS[@]}";
do
  echo "Installing ${JSBSIM_DEP}"
  if which apt-get &> /dev/null; 
  then
    if [[ ! `dpkg -l | grep -w "ii  ${JSBSIM_DEP} "` ]];
    then
      DEB=${DEPS[$JSBSIM_DEP]}
      echo ${DEB}
      wget "${JSBSIM_RELEASE_URL}/${DEB}" --directory-prefix=${JSBSIM_TMP} &> /dev/null
      apt-get install "${JSBSIM_TMP}/${DEB}" 
    fi
  fi
done

# Cleanup tmp dir
rm -r ${JSBSIM_TMP}
