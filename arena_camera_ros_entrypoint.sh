#!/bin/bash

set -e
source /opt/ros/humble/setup.bash
apt update
rosdep update
rosdep install --from-paths src --ignore-src -r -y
./release.bash
export ROS_DOMAIN_ID=1

$@
