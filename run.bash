#!/bin/bash

# Run sequence commands

source /opt/ros/humble/setup.bash
colcon build
source /opt/ros/humble/setup.bash
source install/local_setup.bash

# ros2 run vision predictCam
# ros2 run vision pycam
# ros2 run offboard_control control

gnome-terminal --tab --title='Bridge' -- bash -ic "ros2 run ros_gz_bridge parameter_bridge --ros-args -p config_file:=bridge_config/sensor_bridge.yaml"

# gnome-terminal --tab --title='PyCam' -- bash -ic "ros2 run vision pycam"
gnome-terminal --tab --title='PredCam' -- bash -ic "ros2 run vision predictCam"
gnome-terminal --tab --title='OCGrid' -- bash -ic "ros2 run vision ocgridAdvertiser"
gnome-terminal --tab --title='Control' -- bash -ic "ros2 run offboard_control control"

# SPAWN_X="-25.8"           # -x  UGV spawn X
# SPAWN_Y="16.1"           # -y  UGV spawn Y
# SPAWN_Z="33.3"            # -z  UGV spawn Z (terrain height at spawn)
# gnome-terminal --tab --title='GZ' -- bash -ic "source_ugv_sim; cd $PX4_PATH; HEADLESS=1 PX4_GZ_WORLD=aspa135_m3 PX4_GZ_MODEL_POSE="-22,11,32.8,0,0,0" make px4_sitl gz_x500_segment_cam_down"
# Alignment issues; use default spawn location instead

gnome-terminal --tab --title='GZ' -- bash -ic "source_ugv_sim; cd $PX4_PATH; HEADLESS=1 PX4_GZ_WORLD=aspa135_m3 PX4_GZ_MODEL_POSE="0,0,32.55,0,0,0" make px4_sitl gz_x500_segment_cam_down"
# gnome-terminal --tab --title='GZ' -- bash -ic "source_ugv_sim; cd $PX4_PATH; PX4_GZ_WORLD=aspa135_m3 PX4_GZ_MODEL_POSE="5.05,3.24,32.31,0,0,0" make px4_sitl gz_x500_segment_cam_down"
# gnome-terminal --tab --title='GZ' -- bash -ic "source_ugv_sim; cd $PX4_PATH; PX4_GZ_WORLD=aruco make px4_sitl gz_x500_segment_cam_down"

# gnome-terminal --tab --title='PredHz' -- bash -ic "ros2 topic hz /cam_fps/predictor"

# Husarion
# source ~/Software/qut_uas_ws/install/setup.bash && ros2 launch uas_gazebo_sim rescue_randy_spawner_gz.launch.py world_name:=aspa135_m3
# source_ugv_sim && ros2 launch husarion_ugv_gazebo simulate_robot.launch.py x:=1.0 y:=1.0 z:=34

gnome-terminal --tab --title='UGV' -- bash -ic "sleep 55; source_ugv_sim; source ~/Software/qut_uas_ws/src/uas_gazebo_sim/launch/load_ugv_mission.sh"

gnome-terminal --tab --title='Bag' -- bash -ic "sleep 55; source_ugv_sim; cd ~/Software/ROS_bags/vertical_missions; source ~/Software/qut_uas_ws/install/local_setup.bash; ros2 bag record ugv/global_position"