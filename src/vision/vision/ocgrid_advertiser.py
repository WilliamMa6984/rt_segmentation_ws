#!/usr/bin/env python3

import math

import cv2
import numpy as np

import rclpy
from cv_bridge import CvBridge, CvBridgeError
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
import rclpy.qos
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray
from px4_msgs.msg import VehicleLocalPosition

import matplotlib.pyplot as plt
from ament_index_python.packages import get_package_share_directory
import os

MAP_SZ_M = 60.0
MAP_RESOLUTION = 0.5
DETECTION_SZ = int(MAP_SZ_M / MAP_RESOLUTION)

# WGS-84 Earth radius (metres)
EARTH_RADIUS = 6371000.0

# Gz origin in lon/lat
REF_LAT = -66.28223056
REF_LON = 110.53892500

class OCGridAdvertiser(Node):
    def __init__(self):
        super().__init__('ocgrid_advertiser')

        # Configure QoS profile for publishing and subscribing
        qos_profile = rclpy.qos.QoSProfile(
            reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
            durability=rclpy.qos.DurabilityPolicy.TRANSIENT_LOCAL,
            history=rclpy.qos.HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # self.occupancy_grid_publisher = self.create_publisher(
        #     OccupancyGrid, '/moss_occ_grid', 10
        # )
        self.occupancy_grid_publisher = self.create_publisher(
            Image, '/moss_occ_grid', 10
        )
        self.prediction_subscriber = self.create_subscription(
            Image, '/predictor/image_100', self.predictor_callback,
            qos_profile_sensor_data
        )
        self.pose_subscriber = self.create_subscription(
            Float32MultiArray, '/predictor/robot_pose', self.pose_callback,
            qos_profile_sensor_data
        )
        self.position_subscription = self.create_subscription(
            VehicleLocalPosition, 
            '/fmu/out/vehicle_local_position_v1', 
            self.position_callback, 
            qos_profile)

        self.north = 0.0
        self.east = 0.0
        self.down = 0.0
        self.pitch = 0.0
        self.maxPitch = 0.0
        self.yaw = 0.0
        self.roll = 0.0
        self.maxRoll = 0.0
        self.origin_north = 0.0
        self.origin_east = 0.0
        self.detection_map_img = np.zeros(
            (DETECTION_SZ, DETECTION_SZ), dtype=np.uint8
        )
        self.detection_map_img_historic = np.zeros(
            (DETECTION_SZ, DETECTION_SZ), dtype=np.uint8
        )
        self.detection_map_mask = np.zeros(
            (DETECTION_SZ, DETECTION_SZ), dtype=np.uint8
        )
        self.detection_map_mask_historic = np.zeros(
            (DETECTION_SZ, DETECTION_SZ), dtype=np.uint8
        )
        self.detection_pose = np.zeros((6,1))
        self.detection_pose_countup = 0
        self.bridge = CvBridge()
        self.timer = self.create_timer(0.2, self.publish_occupancy_grid)

        # self.map = cv2.imread(os.path.join(get_package_share_directory('vision'), 'map.png'))
        self.map = cv2.imread(os.path.join(get_package_share_directory('vision'), 'map_mask.png'))
        # self.map = cv2.cvtColor(self.map, cv2.COLOR_BGR2GRAY)

        self.get_logger().info('coord_advertiser node')

        if self.map is None:
            self.get_logger().error("Image failed to load! Check the file path.")

# @brief Subscribe vehicle local position
# @param 
    def position_callback(self, msg):
        if (msg is not None):
            self.origin_north = math.radians(msg.ref_lon - REF_LON) * \
                EARTH_RADIUS * math.cos(math.radians(REF_LAT))
            self.origin_east = math.radians(msg.ref_lat - REF_LAT) * EARTH_RADIUS
            self.get_logger().info(
                f'Target received: ENU ({self.origin_east:.2f}, {self.origin_north:.2f}) '
            )

            self.destroy_subscription(self.position_subscription)
            self.position_subscription = None

# Get robot pose (NED) and orientation (RPY)
    def pose_callback(self, msg):
        if len(msg.data) < 6:
            self.get_logger().warning('robot pose must contain six values')
            return

        self.north, self.east, self.down = msg.data[:3]
        self.roll, self.pitch, self.yaw = msg.data[3:6]

        self.maxPitch = max(self.maxPitch, self.pitch)
        self.maxRoll = max(self.maxRoll, self.roll)

# Get predictor image, transform it, and store it for publishing as an occupancy grid
    def predictor_callback(self, msg):
        print("============")

        # Ignore predictor images if the robot is tilted too much
        if abs(self.pitch) > 0.065 or abs(self.roll) > 0.065:
            return

        try:
            image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='mono8')
        except CvBridgeError:
            self.get_logger().info('cv_bridge exception')
            return

        # Ignore if empty detection or inf dist to ground (out of map)
        if (np.isinf(self.down) or self.down==0 or np.max(image) == 0):
            return

        # # Ignore if pose is similar to previous detection
        # curr_pose = np.array([self.north, self.east, self.down, \
        #                        self.roll, self.pitch, self.yaw])
        # if np.linalg.norm(self.detection_pose-curr_pose) < 0.1: # too close
        #     # self.detection_pose = curr_pose
        #     return
        # else: # continue
        #     self.detection_pose = curr_pose
        
        # Process (rotate + translate) moss seg. image
        self.detection_map_img, self.detection_map_mask = self.rotate_image(
            image,
            -math.degrees(self.yaw),
            self.down,
            self.east+self.origin_east,
            self.north+self.origin_north,
        )

# Rotate and translate the image based on robot pose and lidar distance
    def rotate_image(self, image, angle, dist_from_gnd, tx, ty):
        image_height, image_width = image.shape[:2]
        image_center = (image_width / 2.0, image_height / 2.0)

        fov = 1.45
        focal_length = (image_width * 0.5) / math.tan(fov * 0.5)
        theta = math.atan2(image_width, focal_length)
        projected_width = dist_from_gnd * math.tan(theta)
        scale_factor = projected_width / image_width / MAP_RESOLUTION

        mask = np.ones((image_height, image_width), dtype=np.uint8) * 255

        # Rotate and scale image based on yaw and distance from ground
        rotation_matrix = cv2.getRotationMatrix2D(
            image_center, angle, scale_factor
        )
        rotated_image = cv2.warpAffine(
            image,
            rotation_matrix,
            (image_width, image_height),
            flags=cv2.INTER_LINEAR,
        )
        # Same with mask
        mask = cv2.warpAffine(
            mask,
            rotation_matrix,
            (image_width, image_height),
            flags=cv2.INTER_LINEAR,
        )

        # Translate to map location
        center_x = (DETECTION_SZ - image_width) / 2.0
        center_y = (DETECTION_SZ - image_height) / 2.0
        translation_matrix = np.float32([
            [1, 0, center_x + tx / MAP_RESOLUTION],
            [0, 1, center_y - ty / MAP_RESOLUTION],
        ])
        rotated_image = cv2.warpAffine(
            rotated_image,
            translation_matrix,
            (DETECTION_SZ, DETECTION_SZ),
            flags=cv2.INTER_LINEAR,
        )
        # Same with mask
        mask = cv2.warpAffine(
            mask,
            translation_matrix,
            (DETECTION_SZ, DETECTION_SZ),
            flags=cv2.INTER_LINEAR,
        ) > 0

        return rotated_image, mask

# Publish the occupancy grid based on the transformed predictor image
    def publish_occupancy_grid(self):
        # Weighted sum of historic and current image
        blended = self.detection_map_img*0.1 + self.detection_map_img_historic*0.9

        if (not self.is_similar(blended[self.detection_map_mask], self.detection_map_img_historic[self.detection_map_mask])):
            # Set image to map
            self.detection_map_img_historic[self.detection_map_mask] = blended[self.detection_map_mask]
            # self.detection_map_mask_historic[self.detection_map_mask] = self.detection_map_mask[self.detection_map_mask]

            # Publish message
            # occ_img_msg = self.bridge.cv2_to_imgmsg((self.detection_map_img_historic>20).astype('uint8')*255, encoding="mono8")
            occ_img_msg = self.bridge.cv2_to_imgmsg(self.detection_map_img_historic, encoding="mono8")
            self.occupancy_grid_publisher.publish(occ_img_msg)

        # plt.imshow(self.detection_map_mask_historic)
        # plt.pause(0.05)

        # # Verify against precompute map
        # detection_map_img_ = cv2.cvtColor(self.detection_map_img_historic, cv2.COLOR_GRAY2BGR)
        # detection_map_img_[:, :, 0] = 0  # Blue = 0
        # detection_map_img_[:, :, 2] = 0  # Red = 0

        # blended = cv2.addWeighted(self.map, 0.2, detection_map_img_, 0.8, 0)
        # plt.imshow(blended)
        # plt.pause(0.05)

        self.get_logger().info(
            f'Pos (NED): {self.north} {self.east} {self.down}\n'
            f'RPY: {self.roll} {self.pitch} {self.yaw}\n'
            f'Ref north/east: {self.origin_north} {self.origin_east}\n'
            f'Max tilts (RP): {self.maxRoll} {self.maxPitch}'
        )

# Source - https://stackoverflow.com/a/70112625
# Posted by B-L
# Retrieved 2026-09-24, License - CC BY-SA 4.0
    def is_similar(self, img1, img2):
        #--- take the absolute difference of the images ---
        res = cv2.absdiff(img1.astype(np.uint8),img2.astype(np.uint8))

        #--- convert the result to integer type ---
        res = res.astype(np.uint8)

        #--- find percentage difference based on number of pixels that are not zero ---
        percentage = (np.count_nonzero(res))/ res.size

        print(percentage)
        
        return percentage < 0.2

def main(args=None):
    rclpy.init(args=args)
    node = OCGridAdvertiser()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()