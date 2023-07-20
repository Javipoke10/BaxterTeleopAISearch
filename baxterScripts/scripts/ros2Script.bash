#!/bin/bash

echo "setUp ROS2 Foxy";

cd ~/ros2

echo "Setuping, compiling and running haptic controller and interface";
source ./install/setup.bash
colcon build && ros2 run baxter_controller hapticBaxterControllerUi