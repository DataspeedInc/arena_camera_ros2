#!/bin/bash
docker run -it --name arena_camera_dev --restart always \
  -e "ROS_DOMAIN_ID=1" -e "FASTRTPS_DEFAULT_PROFILES_FILE=/fastrtps-profile.xml" -e "RCUTILS_COLORIZED_OUTPUT=1" \
  -v /dev/shm:/dev/shm --mount src=$PWD/ros2_ws,target=/arena_camera_ros2/ros2_ws,type=bind --net host \
  osrf/ros:humble-desktop-lucid-camera-v.1.0