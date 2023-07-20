#!/bin/bash

cd ~/ros_ws

echo "Running baxter";
./baxter.sh

echo "Unlocking baxter";
rosrun baxter_tools enable_robot.py -e

catkin_make

echo "Running ROS1 controller";
source ./install/setup.bash
rosrun baxter_cont controller.py