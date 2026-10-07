source /opt/ros/humble/setup.bash
colcon build
source ~/ros2_ws/install/setup.bash
source install/local_setup.bash

ros2 run streamer stream_gzcam
