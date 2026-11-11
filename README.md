# Real Time Semantic Segmentation in Realistic Antarctica Gazebo Simulator

## Packages
PX4 Msgs - (git clone https://github.com/PX4/px4_msgs.git)

vision - Computer vision and image processing nodes

streamer - Compressed image transport for distributed ROS2 nodes

offboard_control - UAV control and pathing nodes

## Run scripts
run - executes everything (SITL)

runSim - starts up Gazebo sim + camera/sensor bridge

runDistSim - runSim + runStream (for distributed ROS2)

runStream - runs compressed image transport node for camera frames

runControl - moves UAV -> follows UGV

runVision - runs U-Net semantic segmentation predictor on camera frames (https://github.com/WilliamMa6984/Pytorch-UNet-AntarcticMoss); runs image -> map converter

runUGV - runs UGV mission from ~/Software/qut_uas_ws/src/uas_gazebo_sim; records UGV global position via ROS2 bag

## Branches
HITL:
  - includes runJetson script for full script that is called on the 2nd computer/hardware
  - updates runStream to receive compressed images and republish to new node
