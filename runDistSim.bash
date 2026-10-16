#!/bin/bash

# Run sequence commands

gnome-terminal --tab --title='Bridge' -- bash -ic "ros2 run ros_gz_bridge parameter_bridge --ros-args -p config_file:=bridge_config/sensor_bridge.yaml"

gnome-terminal --tab --title='GZ' -- bash -ic "source_ugv_sim; cd $PX4_PATH; PX4_GZ_WORLD=aspa135_m3 PX4_GZ_MODEL_POSE="0,0,32.5,0,0,0" make px4_sitl gz_x500_segment_cam_down"

gnome-terminal --tab --title='Strean' -- bash -ic "bash runStream.bash"

