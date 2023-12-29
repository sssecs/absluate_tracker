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
        self.subscription_1 = self.create_subscription(
            Image,
            '/camera1/image_raw',
            self.listener_callback_1,
            10)
        self.subscription_2 = self.create_subscription(
            Image,
            '/camera2/image_raw',
            self.listener_callback_2,
            10)
        self.subscription_3 = self.create_subscription(
            Image,
            '/camera3/image_raw',
            self.listener_callback_3,
            10)
        self.subscription_4 = self.create_subscription(
            Image,
            '/camera4/image_raw',
            self.listener_callback_4,
            10)
        
        self.srv = self.create_service(Shoot, 'shoot', self.shoot_callback)

        self.image_count_ = 0

    def shoot_callback(self, request, response):
            self.get_logger().info('Incoming request to shoot a picture')
            cv2.imwrite("/home/sykes/absluate_tracker/data/tracker_calibration_images/camera1/image" + str(self.image_count_) + ".png", self.image_1) 
            cv2.imwrite("/home/sykes/absluate_tracker/data/tracker_calibration_images/camera2/image" + str(self.image_count_) + ".png", self.image_2) 
            cv2.imwrite("/home/sykes/absluate_tracker/data/tracker_calibration_images/camera3/image" + str(self.image_count_) + ".png", self.image_3) 
            cv2.imwrite("/home/sykes/absluate_tracker/data/tracker_calibration_images/camera4/image" + str(self.image_count_) + ".png", self.image_4) 
            response.num = self.image_count_
            self.image_count_ = self.image_count_ + 1
            return response
    
    def listener_callback_1(self, msg):
        br = CvBridge()
        self.image_1 = br.imgmsg_to_cv2(msg)

    def listener_callback_2(self, msg):
        br = CvBridge()
        self.image_2 = br.imgmsg_to_cv2(msg)
    
    def listener_callback_3(self, msg):
        br = CvBridge()
        self.image_3 = br.imgmsg_to_cv2(msg)
    
    def listener_callback_4(self, msg):
        br = CvBridge()
        self.image_4 = br.imgmsg_to_cv2(msg)
    


    
    
    
def main(args=None):
    rclpy.init(args=args)

    image_capture = ImageCapture()

    rclpy.spin(image_capture)

    rclpy.shutdown()


if __name__ == '__main__':
    main()