import sys
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from absluate_tracker_interfaces.srv import Shoot
import cv2 

class ImageCapture(Node):
    def __init__(self):
        super().__init__('image_capture')
        self.subscription = self.create_subscription(
            Image,
            '/camera1/image_raw',
            self.listener_callback,
            10)
        self.srv = self.create_service(Shoot, 'shoot', self.shoot_callback)

    def shoot_callback(self, request, response):
            self.get_logger().info('Incoming request to shoot a picture')
            self.image
            cv2.imwrite("/tmp/image1.png", self.image) 
            return response
    
    def listener_callback(self, msg):
        br = CvBridge()
        self.image = br.imgmsg_to_cv2(msg)
    

    
    
    
def main(args=None):
    rclpy.init(args=args)

    image_capture = ImageCapture()

    rclpy.spin(image_capture)

    rclpy.shutdown()


if __name__ == '__main__':
    main()