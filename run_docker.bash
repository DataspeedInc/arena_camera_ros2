#!/bin/bash
docker run -it --name arena_camera_dev --mount src=$PWD/ros2_ws,target=/arena_camera_ros2/ros2_ws,type=bind --network host osrf/ros:humble-desktop-lucid-camera-v.1.0
