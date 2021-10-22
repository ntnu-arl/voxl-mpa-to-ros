#!/bin/bash
#
# cleans the ros workspace
#
# Modal AI Inc. 2019
# author: james@modalai.com



rm -rf catkin_ws/build/
rm -rf catkin_ws/install/
rm -rf catkin_ws/devel/

rm -rf ipk/control.tar.gz
rm -rf ipk/data/
rm -rf ipk/data.tar.gz
rm -rf *.ipk
