#!/bin/bash

# Run sequence commands

source /opt/ros/humble/setup.bash
colcon build
source /opt/ros/humble/setup.bash
source install/local_setup.bash

gnome-terminal --tab --title='Bridge' -- sh -c "ros2 run ros_gz_bridge parameter_bridge --ros-args -p config_file:=bridge_config/camera_bridge.yaml"

# gnome-terminal --tab --title='Node' -- sh -c "ros2 run px4_pycam talker"
gnome-terminal --tab --title='Node' -- sh -c "ros2 run vision pycam"
# ros2 run vision pycam

# ros2 launch px4_ros_com sensor_combined_listener.launch.py
# ros2 launch px4_ros_com debug_vect_advertiser.launch.py
# ros2 launch px4_ros_com debug_vect_listener.launch.py
# gnome-terminal --tab --title='Control node' -- sh -c "ros2 run px4_ros_com offboard_control"
# ros2 launch px4_ros_com offboard_control_py.launch.py

# export PX4_SITL_WORLD="/home/will/ROS2-workspaces/ws_sensor_combined/worlds/test.sdf"
# export UAV_MODEL=<model-name>
# export UAV_X=<float>  # meters
# export UAV_Y=<float>  # meters
# export UAV_Z=<float>  # meters
# export UAV_YAW=<float>  # radians

gnome-terminal --tab --title='GZ' -- sh -c "cd $HOME/PX4-Autopilot/; PX4_GZ_WORLD=aspa135_m3 PX4_GZ_MODEL_POSE="5.05,3.24,32.31,0,0,0" make px4_sitl gz_x500_segment_cam_down"

# export GZ_SIM_RESOURCE_PATH="/home/will/PX4-Autopilot/Tools/simulation/gz/models/"
#~/PX4-Autopilot/Tools/simulation/gz/worlds$
# gz sim aspa135_m3.sdf

# export GZ_SIM_RESOURCE_PATH='$GZ_SIM_RESOURCE_PATH'
# gnome-terminal --tab --title='GZ' -- sh -c "cd $HOME/PX4-Autopilot/; PX4_GZ_WORLD=test make px4_sitl gz_x500_mono_cam_down"

#initial pose: "5.05,3.24,32.31,0,0,0"
# PX4_GZ_MODEL_POSE="5.05,3.24,32.31,0,0,0"

# make px4_sitl list_vmd_make_targets
# make px4_sitl gz_x500
# make px4_sitl gz_x500_lawn
# make px4_sitl gz_x500_vision
# PX4_GZ_WORLD=aruco make px4_sitl gz_x500_mono_cam_down
# PX4_GZ_WORLD=kthspacelab make px4_sitl gz_x500

# PX4_GZ_WORLD=aruco make px4_sitl gz_x500_mono_cam