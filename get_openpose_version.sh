#!/bin/bash
# author: ravi joshi
# date: 10 may 2020
# this shell script returns the openpose version installed on this machine
# it reads macros.hpp file inside openpose directory

# set the location from first input argument
# set location to "/usr/local" if no argument is given
LOCATION=$1
if [ -z "${LOCATION}" ]; then
  LOCATION="/usr/local"
fi

echo "Looking for installed OpenPose version..."

FILENAME=$(find ${LOCATION} -type f -name "macros.hpp" | awk '/openpose/{print; exit}')
VERSION=$(awk '/OPEN_POSE_VERSION_STRING/{print $5; exit}' ${FILENAME} | awk -F'"' '{print $2;}')

echo "Found OpenPose version ${VERSION}"
echo "OpenPose location ${FILENAME}"

