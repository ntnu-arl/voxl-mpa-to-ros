#!/bin/bash
#
# builds everything without installing
#
# Modal AI Inc. 2019
# author: james@modalai.com


set -e

cd catkin_ws
source /opt/ros/indigo/setup.bash


# this installs to catkin_ws/install
# the make_package.sh script will move into ipk and set permissions manually
# running make install as root is broken with catkin so we have to do it this way

catkin_make install -DCMAKE_BUILD_TYPE=Release -DQC_SOC_TARGET=APQ8096 -Wno-dev

source devel/setup.bash