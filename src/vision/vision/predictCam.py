import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import cProfile

from std_msgs.msg import Bool

import vision.predict as predict
  
class ImagePredictorSubscriber(Node):
  current_frame = None
  model = None
  net = None
  mask_values = [0, 1]
  img_msg = None
  img_100_msg = None # 100x100 image for occupancy grid

  def __init__(self):
    super().__init__('predictor_subscriber')
    self.subscription = self.create_subscription(
      Image, 
      '/camera/image', 
      self.listener_callback, 
      10)
    self.subscription # prevent unused variable warning
    self.publisher_ = self.create_publisher(Bool, '/predictor/fps', 10)
    self.img_publisher_ = self.create_publisher(Image, '/predictor/image', 10)
    self.img_100_publisher_ = self.create_publisher(Image, '/predictor/image_100', 10)
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

    # cv2.imshow("camera", mask)
    # cv2.waitKey(1)

    # self.img_msg = self.br.cv2_to_imgmsg(mask, encoding="mono8")
    self.img_100_msg = self.br.cv2_to_imgmsg(cv2.resize(mask, (100, 100)) , encoding="mono8")
    
    msg = Bool()
    msg.data = True
    self.publisher_.publish(msg)
    
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