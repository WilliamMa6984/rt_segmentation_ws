import rclpy
import rclpy.qos as QoS
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import cProfile
from scipy.spatial.transform import Rotation as R

# from std_msgs.msg import Bool
from std_msgs.msg import Float32MultiArray
from px4_msgs.msg import VehicleLocalPosition
from px4_msgs.msg import VehicleAttitude

import vision.predict as predict
  
class ImagePredictorSubscriber(Node):
  current_frame = None
  model = None
  net = None
  mask_values = [0, 1]
  img_msg = None
  img_100_msg = None # 100x100 image for occupancy grid
  pose_msg = None # 100x100 image for occupancy grid
  north = 0.0
  east = 0.0
  down = 0.0
  roll = 0.0
  pitch = 0.0
  yaw = 0.0

  def __init__(self):
    super().__init__('predictor_subscriber')

    # Configure QoS profile for publishing and subscribing
    qos_profile = QoS.QoSProfile(
        reliability=QoS.ReliabilityPolicy.BEST_EFFORT,
        durability=QoS.DurabilityPolicy.TRANSIENT_LOCAL,
        history=QoS.HistoryPolicy.KEEP_LAST,
        depth=1
    )

    self.subscription = self.create_subscription(
      Image, 
      '/camera/image', 
      self.listener_callback, 
      10)
    self.subscription # prevent unused variable warning
    self.position_subscription = self.create_subscription(
      VehicleLocalPosition, 
      '/fmu/out/vehicle_local_position_v1', 
      self.position_callback, 
      qos_profile)
    self.position_subscription # prevent unused variable warning
    self.attitude_subscription = self.create_subscription(
      VehicleAttitude, 
      '/fmu/out/vehicle_attitude', 
      self.attitude_callback, 
      qos_profile)
    self.attitude_subscription # prevent unused variable warning
    # self.publisher_ = self.create_publisher(Bool, '/predictor/fps', 10)
    self.img_publisher_ = self.create_publisher(Image, '/predictor/image', 10)
    self.img_100_publisher_ = self.create_publisher(Image, '/predictor/image_100', 10)
    self.robot_pose_publisher_ = self.create_publisher(Float32MultiArray, '/predictor/robot_pose', 10)
    self.br = CvBridge()

    self.net, self.mask_values, self.device = predict.unet_load()
    if (self.net):
      self.get_logger().info("Model loaded: " + str(self.mask_values))

    self.timer = self.create_timer(0.1, self.publisher_callback)
  
    print("predictCam node")
    
  def publisher_callback(self):
    # if (self.img_msg is not None):
    #   self.img_publisher_.publish(self.img_msg)
    if (self.img_100_msg is not None):
      self.img_100_publisher_.publish(self.img_100_msg)
    if (self.pose_msg is not None):
      self.robot_pose_publisher_.publish(self.pose_msg)
      # np.set_printoptions(precision=2, suppress=True)
      # print(np.array(self.pose_msg.data))


  def listener_callback(self, data):
    img = self.br.imgmsg_to_cv2(data) # PIL image uses RGB, don't convert to BGR

    # Reshape to 572, 572
    # Source - https://stackoverflow.com/a/61942452
    # Posted by Juan Esteban Fonseca, modified by community. See post 'Timeline' for change history
    # Retrieved 2026-05-17, License - CC BY-SA 4.0
    w = 960
    h = 960

    center = img.shape
    x = center[1]/2 - w/2
    y = center[0]/2 - h/2

    img = img[int(y):int(y+h), int(x):int(x+w)]

    self.current_frame = cv2.resize(img, (572, 572)) 

    mask = predict.predict_img(net=self.net,
                    full_img=self.current_frame,
                    scale_factor=1.0,
                    out_threshold=0.2,
                    device=self.device)
    
    mask = mask.astype(np.uint8)*100
    self.pose_msg = Float32MultiArray()
    self.pose_msg.data = [self.north, self.east, self.down,
                         self.roll, self.pitch, self.yaw]

    # cv2.imshow("camera", mask)
    # cv2.waitKey(1)

    # self.img_msg = self.br.cv2_to_imgmsg(mask, encoding="mono8")
    self.img_100_msg = self.br.cv2_to_imgmsg(cv2.resize(mask, (100, 100)) , encoding="mono8")
    
    # msg = Bool()
    # msg.data = True
    # self.publisher_.publish(msg)
    

# @brief Subscribe vehicle local position
# @param 
  def position_callback(self, msg):
    if (msg is not None):
      self.north = msg.x
      self.east = msg.y
      self.down = msg.z
    
# @brief Subscribe vehicle attitude
# @param 
  def attitude_callback(self, msg):
    if (msg is not None):
      # Quaternion: w x y z
      rot = R.from_quat([msg.q[1], msg.q[2], msg.q[3], msg.q[0]]);
      
      self.roll, self.pitch, self.yaw = rot.as_euler('xyz');

# def process(args=None):
#   rclpy.init(args=args)
#   image_subscriber = ImagePredictorSubscriber()
#   rclpy.spin(image_subscriber)
#   image_subscriber.destroy_node()
#   rclpy.shutdown()

def main(args=None):
  # cProfile.runctx('process(args)', globals(), locals(), 'profile')
  rclpy.init(args=args)
  image_subscriber = ImagePredictorSubscriber()
  rclpy.spin(image_subscriber)
  image_subscriber.destroy_node()
  rclpy.shutdown()
   
if __name__ == '__main__':
  main()