import sys
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from absluate_tracker_interfaces.srv import Shoot
import cv2 
import numpy as np
import pickle

class ImageCapture(Node):
    def __init__(self):
        super().__init__('image_capture')
        self.subscription_1_ = self.create_subscription(
            Image,
            '/camera1/image_raw',
            self.listener_callback_1,
            10)
        self.subscription_2_ = self.create_subscription(
            Image,
            '/camera2/image_raw',
            self.listener_callback_2,
            10)
        self.subscription_3_ = self.create_subscription(
            Image,
            '/camera3/image_raw',
            self.listener_callback_3,
            10)
        self.subscription_4_ = self.create_subscription(
            Image,
            '/camera4/image_raw',
            self.listener_callback_4,
            10)
        
        self.srv_ = self.create_service(Shoot, 'shoot', self.shoot_callback)

        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        

        self.images_ = [np.zeros((1920,1080,3), dtype=np.uint8),np.zeros((1920,1080,3), dtype=np.uint8),np.zeros((1920,1080,3), dtype=np.uint8),np.zeros((1920,1080,3), dtype=np.uint8)]

        self.coord_ = np.array([[0.0, 0.0], [0.0, 0.0], [0.0, 0.0], [0.0, 0.0]])
        self.coord_history_ = np.array([[0.0, 0.0], [0.0, 0.0], [0.0, 0.0], [0.0, 0.0]])
        self.coord_composed_ = []

        self.image_count_ = 0
        self.coord_count_ = 0


    def shoot_callback(self, request, response):
            self.get_logger().info('Incoming request to shoot a picture')
            cv2.imwrite("/home/sykes/absluate_tracker/data/tracker_calibration_images/camera1/image" + str(self.image_count_) + ".png", self.images_[0]) 
            cv2.imwrite("/home/sykes/absluate_tracker/data/tracker_calibration_images/camera2/image" + str(self.image_count_) + ".png", self.images_[1]) 
            cv2.imwrite("/home/sykes/absluate_tracker/data/tracker_calibration_images/camera3/image" + str(self.image_count_) + ".png", self.images_[2]) 
            cv2.imwrite("/home/sykes/absluate_tracker/data/tracker_calibration_images/camera4/image" + str(self.image_count_) + ".png", self.images_[3]) 
            response.num = self.image_count_
            self.image_count_ = self.image_count_ + 1
            return response
    
    def listener_callback_1(self, msg):
        br = CvBridge()
        self.images_[0] = br.imgmsg_to_cv2(msg)        

    def listener_callback_2(self, msg):
        br = CvBridge()
        self.images_[1] = br.imgmsg_to_cv2(msg)
    
    def listener_callback_3(self, msg):
        br = CvBridge()
        self.images_[2] = br.imgmsg_to_cv2(msg)
    
    def listener_callback_4(self, msg):
        br = CvBridge()
        self.images_[3] = br.imgmsg_to_cv2(msg)
    
    def detect_target(self,image):
        image_r,g,b = cv2.split(image)
        img = cv2.inRange(image_r,240,255)
        contours, hierarchy = cv2.findContours(img,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
        if len(contours) != 0:
            c = contours[-1]
            M = cv2.moments(c)
            if M["m00"] != 0:
                cX = int(M["m10"] / M["m00"])
                cY = int(M["m01"] / M["m00"])
            else:
               cX = cY = -1 
        else:
            cX = cY = -1
        return (cX,cY)
    



    def target_ball_update(self):
        for i in range(4):
            self.coord_[i,0], self.coord_[i,1] = self.detect_target(self.images_[i])

        if not np.any(self.coord_ == -1):
            if np.linalg.norm(self.coord_[0] - self.coord_history_[0]) > 5 or np.linalg.norm(self.coord_[1] - self.coord_history_[1]) > 5 or np.linalg.norm(self.coord_[2] - self.coord_history_[2]) > 5 or np.linalg.norm(self.coord_[3] - self.coord_history_[3]) > 5:
                self.coord_history_ = self.coord_.copy()
                self.coord_composed_.append(self.coord_history_)
                self.get_logger().info('coord recorded: "%s"' % self.coord_count_)
                self.coord_count_ = self.coord_count_ + 1
                self.save_object(self.coord_composed_)
                
    def save_object(self,data):
        try:
            with open("/home/sykes/absluate_tracker/data/data.pickle", "wb") as f:
                pickle.dump(data, f, protocol=pickle.HIGHEST_PROTOCOL)
        except Exception as ex:
            print("Error during pickling object (Possibly unsupported):", ex)

    def timer_callback(self):
        self.target_ball_update()

    
def main(args=None):
    rclpy.init(args=args)

    image_capture = ImageCapture()

    rclpy.spin(image_capture)

    rclpy.shutdown()


if __name__ == '__main__':
    main()