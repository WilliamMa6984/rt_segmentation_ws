import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
from rclpy.time import Time

from std_msgs.msg import Bool

import vision.predict as predict
  
class ImagePredictorSubscriber(Node):
  current_frame = None
  model = None
  net = None
  mask_values = [0, 1]

  def __init__(self):
    super().__init__('predictor_subscriber')
    self.subscription = self.create_subscription(
      Image, 
      '/camera/image', 
      self.listener_callback, 
      10)
    self.subscription # prevent unused variable warning
    self.br = CvBridge()

    self.net, self.mask_values, self.device = predict.unet_load()
    if (self.net):
      self.get_logger().info("Model loaded: " + str(self.mask_values))
    
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

    cv2.imshow("camera", mask.astype(np.uint8)*255)
    cv2.waitKey(1)
    

def main(args=None):
  rclpy.init(args=args)
  image_subscriber = ImagePredictorSubscriber()
  rclpy.spin(image_subscriber)
  image_subscriber.destroy_node()
  rclpy.shutdown()
   
if __name__ == '__main__':
  main()