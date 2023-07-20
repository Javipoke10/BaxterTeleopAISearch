#!/bin/bash

echo "setUp ROS2 Foxy";
rosconfig -d foxy -w /home/javier/ros2/

echo "SetUp ROS1 Noetic";
rosconfig -d noetic -w /home/javier/ros_ws/ -m 192.168.1.160  -i "ip de tu red conectada al baxter"

echo "Launching bridge";
ros2 run ros1_bridge dynamic_bridge