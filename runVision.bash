#!/bin/bash

# Run sequence commands

source /opt/ros/humble/setup.bash
colcon build
source /opt/ros/humble/setup.bash
source install/local_setup.bash

gnome-terminal --tab --title='Bridge' -- bash -ic "ros2 run ros_gz_bridge parameter_bridge --ros-args -p config_file:=bridge_config/sensor_bridge.yaml"

gnome-terminal --tab --title='GZ' -- bash -ic "cd $PX4_PATH; HEADLESS=1 PX4_GZ_WORLD=aspa135_m3 PX4_GZ_MODEL_POSE="5.05,3.24,32.31,0,0,0" make px4_sitl gz_x500_segment_cam_down"

# gnome-terminal --tab --title='GZ' -- bash -ic "cd $PX4_PATH; HEADLESS=1 PX4_GZ_WORLD=robbos PX4_GZ_MODEL_POSE="-9.0,-10.0,32.36,0,0,0" make px4_sitl gz_x500_segment_cam_down"

# ros2 run vision predictCam
ros2 run vision pycam
