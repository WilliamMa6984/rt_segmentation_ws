#!/bin/bash

# Run sequence commands

source /opt/ros/humble/setup.bash
colcon build
source /opt/ros/humble/setup.bash
source install/local_setup.bash

gnome-terminal --tab --title='UGV' -- bash -ic "source_ugv_sim; source ~/Software/qut_uas_ws/src/uas_gazebo_sim/launch/load_ugv_mission.sh"
gnome-terminal --tab --title='Bag' -- bash -ic "source_ugv_sim; cd ~/ROS_bags/v2/horizontal; source ~/Software/qut_uas_ws/install/local_setup.bash; ros2 bag record ugv/global_position"
