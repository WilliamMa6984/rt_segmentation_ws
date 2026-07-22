#!/bin/bash

# Run sequence commands

source /opt/ros/humble/setup.bash
colcon build
source /opt/ros/humble/setup.bash
source install/local_setup.bash

# gnome-terminal --tab --title='Bridge' -- sh -c "ros2 run ros_gz_bridge parameter_bridge --ros-args -p config_file:=bridge_config/sensor_bridge.yaml"

# ros2 run offboard_control control
ros2 run offboard_control coord_advertiser
