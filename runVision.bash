#!/bin/bash

# Run sequence commands

source /opt/ros/humble/setup.bash
colcon build
source /opt/ros/humble/setup.bash
source install/local_setup.bash

ros2 run vision predictCam
