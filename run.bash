#!/bin/bash

# Run sequence commands

source /opt/ros/humble/setup.bash
colcon build
source /opt/ros/humble/setup.bash
source install/local_setup.bash

gnome-terminal --tab --title='Bridge' -- sh -c "ros2 run ros_gz_bridge parameter_bridge --ros-args -p config_file:=bridge_config/camera_bridge.yaml"

gnome-terminal --tab --title='Node' -- sh -c "ros2 run vision pycam"
# ros2 run vision pycam

gnome-terminal --tab --title='Node' -- sh -c "ros2 run offboard_control control"
# ros2 run offboard_control control

gnome-terminal --tab --title='GZ' -- sh -c "cd $HOME/PX4-Autopilot/; PX4_GZ_WORLD=aspa135_m3 PX4_GZ_MODEL_POSE="5.05,3.24,32.31,0,0,0" make px4_sitl gz_x500_segment_cam_down"
