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
from sensor_msgs.msg import Image, LaserScan
from std_msgs.msg import Float32MultiArray
from px4_msgs.msg import VehicleLocalPosition


MAP_SZ_M = 60.0
MAP_RESOLUTION = 0.5
DETECTION_SZ = int(MAP_SZ_M / MAP_RESOLUTION)

# WGS-84 Earth radius (metres)
EARTH_RADIUS = 6371000.0

# Bottom left
# REF_LAT = -66.28249195481575
# REF_LON = 110.53827323985684
# lat: -66.28249195481575
# lon: 110.53827323985684
# Middle
REF_LAT = -66.28224326862865
REF_LON = 110.53892436645867
# lat: -66.28224326862865
# lon: 110.53892436645867


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
        self.lidar_subscriber = self.create_subscription(
            LaserScan, '/lidar', self.lidar_callback, qos_profile_sensor_data
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
        self.yaw = 0.0
        self.roll = 0.0
        self.origin_north = 0.0
        self.origin_east = 0.0
        self.lidar_dist = 0.0
        self.detection_map = np.zeros(DETECTION_SZ * DETECTION_SZ, dtype=np.uint8)
        self.detection_map_img = np.zeros(
            (DETECTION_SZ, DETECTION_SZ), dtype=np.uint8
        )
        self.bridge = CvBridge()
        self.timer = self.create_timer(0.2, self.publish_occupancy_grid)

        self.get_logger().info('coord_advertiser node')

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

# Get lidar distance to ground
    def lidar_callback(self, msg):
        if msg.ranges:
            self.lidar_dist = msg.ranges[0]

# Get robot pose (NED) and orientation (RPY)
    def pose_callback(self, msg):
        if len(msg.data) < 6:
            self.get_logger().warning('robot pose must contain six values')
            return

        self.north, self.east, self.down = msg.data[:3]
        self.roll, self.pitch, self.yaw = msg.data[3:6]

# Get predictor image, transform it, and store it for publishing as an occupancy grid
    def predictor_callback(self, msg):
        # Ignore predictor images if the robot is tilted too much
        # if abs(self.pitch) > 0.13 or abs(self.roll) > 0.13:
        #     return

        try:
            image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='mono8')
        except CvBridgeError:
            self.get_logger().info('cv_bridge exception')
            return

        self.detection_map_img = self.rotate_image(
            image,
            -math.degrees(self.yaw),
            self.lidar_dist,
            self.east+self.origin_east,
            self.north+self.origin_north,
        ) # -(DETECTION_SZ*MAP_RESOLUTION/2)
        # 5.05,3.24

# Rotate and translate the image based on robot pose and lidar distance
    def rotate_image(self, image, angle, dist_from_gnd, tx, ty):
        image_height, image_width = image.shape[:2]
        image_center = (image_width / 2.0, image_height / 2.0)

        fov = 1.74
        focal_length = (image_width * 0.5) / math.tan(fov * 0.5)
        theta = math.atan2(image_width, focal_length)
        projected_width = dist_from_gnd * math.tan(theta)
        scale_factor = projected_width / image_width / MAP_RESOLUTION

        rotation_matrix = cv2.getRotationMatrix2D(
            image_center, angle, scale_factor
        )
        rotated_image = cv2.warpAffine(
            image,
            rotation_matrix,
            (image_width, image_height),
            flags=cv2.INTER_LINEAR,
        )

        center_x = (DETECTION_SZ - image_width) / 2.0
        center_y = (DETECTION_SZ - image_height) / 2.0
        translation_matrix = np.float32([
            [1, 0, center_x + tx / MAP_RESOLUTION],
            [0, 1, center_y - ty / MAP_RESOLUTION],
        ])
        return cv2.warpAffine(
            rotated_image,
            translation_matrix,
            (DETECTION_SZ, DETECTION_SZ),
            flags=cv2.INTER_LINEAR,
        )

# Publish the occupancy grid based on the transformed predictor image
    def publish_occupancy_grid(self):
        # flattened_image = self.detection_map_img.reshape(-1)
        # copy_size = min(flattened_image.size, self.detection_map.size)
        # self.detection_map[:copy_size] = flattened_image[:copy_size]

        # occupancy_grid = OccupancyGrid()
        # occupancy_grid.header.stamp = self.get_clock().now().to_msg()
        # occupancy_grid.header.frame_id = 'map'
        # occupancy_grid.info.resolution = MAP_RESOLUTION
        # occupancy_grid.info.width = DETECTION_SZ
        # occupancy_grid.info.height = DETECTION_SZ
        # occupancy_grid.info.origin.position.x = 0.0
        # occupancy_grid.info.origin.position.y = 0.0
        # occupancy_grid.info.origin.position.z = 0.0
        # occupancy_grid.info.origin.orientation.w = 0.0
        # occupancy_grid.data = self.detection_map.view(np.int8).tolist()
        # self.occupancy_grid_publisher.publish(occupancy_grid)
        occ_img_msg = self.bridge.cv2_to_imgmsg(self.detection_map_img , encoding="mono8")
        self.occupancy_grid_publisher.publish(occ_img_msg)

        self.get_logger().info(
            f'Pos (NED): {self.north} {self.east} {self.down}\n'
            f'RPY: {self.roll} {self.pitch} {self.yaw}\n'
            f'Ref north/east: {self.origin_north} {self.origin_east}\n'
            f'Dist to gnd: {self.lidar_dist}'
        )


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