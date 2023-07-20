#!/bin/bash

cd ~/ros2

echo "setUp ROS2 Foxy";
source ./install/setup.bash

echo "Launching realsense";
ros2 launch realsense2_camera rs_launch.py pointcloud.enable:=true

echo "Running vision IA";
ros2 launch cv_basics image_visualizer_launch.py