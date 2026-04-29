import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
from datetime import datetime
import os
  
class ImageSubscriber(Node):
  current_frame = None
  timer_save_img_period = 0.5
  saved_img_folder = ""
  
  def __init__(self):
    super().__init__('image_subscriber')
    self.subscription = self.create_subscription(
      Image, 
      '/camera/image', 
      self.listener_callback, 
      10)
    self.subscription # prevent unused variable warning
    self.br = CvBridge()
    
    t = datetime.now()
    self.saved_img_folder = t.isoformat(timespec='milliseconds')
    os.makedirs(os.path.join("./out", self.saved_img_folder), exist_ok=True)
    self.create_timer(self.timer_save_img_period, self.save_image_process)
    
  def listener_callback(self, data):
    self.current_frame = self.br.imgmsg_to_cv2(data)
    cv2.imshow("camera", self.current_frame)
    cv2.waitKey(1)
    
  def save_image_process(self):
    if (self.current_frame is not None):
      t = datetime.now()
      s = t.isoformat(timespec='milliseconds')
      filename = os.path.join("./out", self.saved_img_folder, s+".jpg")
      print(filename)
      cv2.imwrite(filename, self.current_frame)

def main(args=None):
  rclpy.init(args=args)
  image_subscriber = ImageSubscriber()
  rclpy.spin(image_subscriber)
  image_subscriber.destroy_node()
  rclpy.shutdown()
   
if __name__ == '__main__':
  main()