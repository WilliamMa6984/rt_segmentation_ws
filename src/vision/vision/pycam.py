import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import queue

q = queue.Queue()
  
class ImageSubscriber(Node):
  image = None
  
  def __init__(self):
    super().__init__('image_subscriber')
    self.subscription = self.create_subscription(
      Image, 
      '/camera/image', 
      self.listener_callback, 
      10)
    self.subscription # prevent unused variable warning
    self.br = CvBridge()

    # self.worker = self.create_service(
    #   Image, 
    #   '/camera/image', 
    #   self.listener_callback, 
    #   10)
    # self.subscription # prevent unused variable warning
    
  def listener_callback(self, data):
    # self.get_logger().info('Receiving video frame')
    current_frame = self.br.imgmsg_to_cv2(data)
    # mask = cv2.inRange(current_frame, (0,0,0), (20,20,20))
    # q.put(mask)
    
    cv2.imshow("camera", current_frame)
    cv2.waitKey(1)

    # detector = cv2.SimpleBlobDetector()
    # keypoints = detector.detect(mask)
   
def image_processor(im):
  detector = cv2.SimpleBlobDetector()
  keypoints = detector.detect(im)
  # im_with_keypoints = cv2.drawKeypoints(im, keypoints, np.array([]), (0,0,255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
  # cv2.imshow("camera", im_with_keypoints)
  # cv2.waitKey(1)
 

def main(args=None):
  rclpy.init(args=args)
  image_subscriber = ImageSubscriber()
  rclpy.spin(image_subscriber)
  image_subscriber.destroy_node()
  rclpy.shutdown()
   
if __name__ == '__main__':
  main()