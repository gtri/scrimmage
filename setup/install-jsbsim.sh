#!/bin/bash

JSBSIM_RELEASE_URL=https://github.com/JSBSim-Team/jsbsim/releases/download/v1.2.0
JSBSIM_DEVEL=JSBSIM-devel_1.2.0-1191.jammy.amd64.deb
JSBSIM_EXEC=JSBSIM_1.2.0-1191.jammy.amd64.deb

declare -A DEPS
DEPS[jsbsim-devel]=${JSBSIM_DEVEL}
DEPS[jsbsim]=${JSBSIM_EXEC}

which apt-get &> /dev/null

for JSBSIM_DEP in "${!DEPS[@]}";
do
  if which apt-get &> /dev/null; 
  then
    if [[ ! `dpkg -l | grep -w "ii  ${JSBSIM_DEP} "` ]];
    then
      DEB=${DEPS[$JSBSIM_DEP]}
      echo ${DEB}
      wget "${JSBSIM_RELEASE_URL}/${DEB}" &> /dev/null
      apt-get install "./${DEB}" 
      rm "${DEB}"
    fi
  fi
done
