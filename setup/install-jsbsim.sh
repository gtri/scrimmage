#!/bin/bash

SCRIMMAGE_ROOT=/opt/scrimmage
JSBSIM_ROOT=${SCRIMMAGE_ROOT}/etc/jsbsim
JSBSIM_TMP=${JSBSIM_ROOT}/tmp

JSBSIM_VERSION=1.2.0
JSBSIM_RELEASE_URL=https://github.com/JSBSim-Team/jsbsim/releases/download/v${JSBSIM_VERSION}
JSBSIM_SRC_URL=https://github.com/JSBSim-Team/jsbsim/archive/refs/tags
JSBSIM_DEVEL=JSBSIM-devel_1.2.0-1191.jammy.amd64.deb
JSBSIM_EXEC=JSBSIM_1.2.0-1191.jammy.amd64.deb
JSBSIM_ARCHIVE=v${JSBSIM_VERSION}.tar.gz

declare -A DEPS
DEPS[jsbsim-devel]=${JSBSIM_DEVEL}
DEPS[jsbsim]=${JSBSIM_EXEC}

which apt-get &> /dev/null

CLEANUP=0

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


if [[ ! `ls | grep -w ${JSBSIM_TMP}/${JSBSIM_ARCHIVE}` ]];
then
    wget ${JSBSIM_SRC_URL}/${JSBSIM_ARCHIVE} --directory-prefix=${JSBSIM_TMP} &> /dev/null
fi

EXTRACT_LIST=(
    aircraft
    data_output
    data_plot
    engine
    scripts
    systems
)

# Remove specifc contents of JSBSIM (primaraly aircraft and script data)
# and put it in ${JSBSIM_ROOT}
for TO_EXTRACT in "${EXTRACT_LIST[@]}"; 
do
    tar -C ${JSBSIM_ROOT} -xzf ${JSBSIM_TMP}/${JSBSIM_ARCHIVE} jsbsim-${JSBSIM_VERSION}/${TO_EXTRACT} \
        --strip-components 1
done

# Cleanup tmp dir
rm -r ${JSBSIM_TMP}

# Link custom scrimmage aircraft + scripts so that JSBSIM can find them
DATA_DIRS_TO_LINK=(
    aircraft
    scripts
    )

for DIR in "${DATA_DIRS_TO_LINK[@]}";
do
    DIR_ABS_PATH=$(find /home -wholename "*scrimmage/data/${DIR}")
    for TO_LN in $(ls ${DIR_ABS_PATH});
    do
        ln -s ${DIR_ABS_PATH}/${TO_LN} ${JSBSIM_ROOT}/${DIR}/${TO_LN} 
    done
done
