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

        self.publisher_1_ = self.create_publisher(Image, '/camera1/image_marked', 10)
        
        self.srv_ = self.create_service(Shoot, 'shoot', self.shoot_callback)




        self.image_count_ = 0


    def shoot_callback(self, request, response):
            self.get_logger().info('Incoming request to shoot a picture')
            cv2.imwrite("/home/sykes/absluate_tracker/data/tracker_calibration_images/camera1/image" + str(self.image_count_) + ".png", self.image_1_) 
            cv2.imwrite("/home/sykes/absluate_tracker/data/tracker_calibration_images/camera2/image" + str(self.image_count_) + ".png", self.image_2_) 
            cv2.imwrite("/home/sykes/absluate_tracker/data/tracker_calibration_images/camera3/image" + str(self.image_count_) + ".png", self.image_3_) 
            cv2.imwrite("/home/sykes/absluate_tracker/data/tracker_calibration_images/camera4/image" + str(self.image_count_) + ".png", self.image_4_) 
            response.num = self.image_count_
            self.image_count_ = self.image_count_ + 1
            return response
    
    def listener_callback_1(self, msg):
        br = CvBridge()
        self.image_1_ = br.imgmsg_to_cv2(msg)
        self.marked_msg_1_ = br.cv2_to_imgmsg(self.detect_target(self.image_1_))
        self.publisher_1_.publish(self.marked_msg_1_)

    def listener_callback_2(self, msg):
        br = CvBridge()
        self.image_2_ = br.imgmsg_to_cv2(msg)
    
    def listener_callback_3(self, msg):
        br = CvBridge()
        self.image_3_ = br.imgmsg_to_cv2(msg)
    
    def listener_callback_4(self, msg):
        br = CvBridge()
        self.image_4_ = br.imgmsg_to_cv2(msg)
    
    def detect_target(self,image):
        image_r,g,b = cv2.split(image)
        img = cv2.inRange(image_r,240,255)
        contours, hierarchy = cv2.findContours(img,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)

        if len(contours) != 0:
            c = contours[-1]
            # calculate moments for each contour
            M = cv2.moments(c)

            # calculate x,y coordinate of center
            cX = int(M["m10"] / M["m00"])
            cY = int(M["m01"] / M["m00"])
            image_marked = image.copy()
            cv2.circle(image_marked, (cX, cY), 5, (255, 255, 255), -1)
            cv2.putText(image_marked, "centroid", (cX -25, cY -25),cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
        else:
            image_marked = image.copy()
        return image_marked


    
    
    
def main(args=None):
    rclpy.init(args=args)

    image_capture = ImageCapture()

    rclpy.spin(image_capture)

    rclpy.shutdown()


if __name__ == '__main__':
    main()