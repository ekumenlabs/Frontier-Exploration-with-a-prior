#!/bin/bash

CPU=$(lscpu | grep -oP 'Architecture:\s*\K.+')

SCRIPT_PATH="$( cd "$(dirname "$0")" ; pwd -P )"
SRC_PATH="$( realpath ${SCRIPT_PATH}/../.. )"

# Colors
RED='\033[0;31m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

ARM_IGNORED_PKGS=("ca_gazebo" "ca_imu")

# If the CPU architecture is an ARM (Raspberry) don't compile some packages.
if [ "$CPU" == *"arm"* ] ; then
    echo -e "${YELLOW}Skipping compilation of some packages because of ARM architecure.${NC}"
    
    for pkg in ${ARM_IGNORED_PKGS} ; do
        touch "${SRC_PATH}/${pkg}/CATKIN_IGNORE"
    done
fi
