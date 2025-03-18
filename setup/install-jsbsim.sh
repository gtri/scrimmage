#!/bin/bash

SCRIMMAGE_ROOT=/opt/scrimmage
JSBSIM_ROOT=${SCRIMMAGE_ROOT}/etc/jsbsim
JSBSIM_TMP=${JSBSIM_ROOT}/tmp

LSB_RELEASE=$(lsb_release -sc)
JSBSIM_VERSION=1.2.0
JSBSIM_SUBVERSION=1191
JSBSIM_RELEASE_URL=https://github.com/JSBSim-Team/jsbsim/releases/download/v${JSBSIM_VERSION}
JSBSIM_DEVEL=JSBSIM-devel_${JSBSIM_VERSION}-${JSBSIM_SUBVERSION}.${LSB_RELEASE}.amd64.deb
JSBSIM_BIN=JSBSIM_${JSBSIM_VERSION}-${JSBSIM_SUBVERSION}.${LSB_RELEASE}.amd64.deb

declare -A DEPS
DEPS[jsbsim-devel]=${JSBSIM_DEVEL}
DEPS[jsbsim]=${JSBSIM_BIN}

which apt-get &> /dev/null

# Create tmp dir. If it exists, error so we don't overwrite anything
if [ ! -d "${JSBSIM_TMP}" ];
then
  mkdir -p ${JSBSIM_TMP} || { echo "Failed to create ${JSBSIM_TMP}. Are you root?"; exit 1; }
else
    echo "Temporary directory ${JSBSIM_TMP} already exists. Please remove 
    to proceed with installation of JSBSIM" 
    exit 1
fi

# Install jsbsim & jsbsim-devl from deb packages
for JSBSIM_DEP in "${!DEPS[@]}";
do
  printf "Installing ${JSBSIM_DEP}...\n"
  if which apt-get &> /dev/null; 
  then
    DEB=${DEPS[$JSBSIM_DEP]}
    if [[ ! `dpkg -l | grep -w "ii  ${JSBSIM_DEP} "` ]];
    then
      printf ${DEB}
      wget "${JSBSIM_RELEASE_URL}/${DEB}" --directory-prefix=${JSBSIM_TMP} &> /dev/null
      apt-get install "${JSBSIM_TMP}/${DEB}" 
    else
      printf "${DEB} is already installed on this system!\n"
    fi
  fi
  printf "\n"
done

# Cleanup tmp dir.
if [ -f ${JSBSIM_TMP}/${JSBSIM_DEVEL} ]; then 
  rm ${JSBSIM_TMP}/${JSBSIM_DEVEL}
fi

if [ -f ${JSBSIM_TMP}/${JSBSIM_BIN} ]; then 
  rm ${JSBSIM_TMP}/${JSBSIM_BIN}
fi

# Will only remove directory if it is already empty
rmdir ${JSBSIM_TMP}
