#!/bin/bash

echo "setUp ROS2 Foxy";
source ./install/setup.bash

echo "Running the TF";
ros2 run tf2_ros static_transform_publisher 0.22 0.0 0.8 0.0 0.95 0.0 base camera_link