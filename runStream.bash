source /opt/ros/humble/setup.bash
colcon build
source ~/ros2_ws/install/setup.bash
source install/local_setup.bash

#ros2 run streamer stream_gzcam
ros2 run image_transport republish compressed raw --ros-args --remap in/compressed:=/camera/stream/image/compressed --remap out:=/camera/decomp/image
