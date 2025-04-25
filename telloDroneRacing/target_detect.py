import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from rclpy.qos import QoSProfile, QoSReliabilityPolicy

import numpy as np
import cv2
from cv_bridge import CvBridge



class TargetDetection(Node):

    def __init__(self):
        super().__init__('gate_detector')
        
        self.bridge = CvBridge()


        #Tello drone/sim require naming a policy for connection
        qos_profile = QoSProfile(depth=10)
        qos_profile.reliability = QoSReliabilityPolicy.BEST_EFFORT
        
        self.publisher = self.create_publisher(Image,'gates',qos_profile)
        
        ##Same as image_proc, which might not be needed on the physical drone
        self.subscription = self.create_subscription(
            Image,
            '/drone1/image_raw',
            self.listener_callback,
            qos_profile)
        #'/drone1/image_raw'
        #'/image_raw'

        self.get_logger().info("Target_detection node has been started.")
        ##^^


    def listener_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            #Gate detection here?
            #Pub//Call planning?
        
            #gray=cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

        
            #Detect green gate?
            color=cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
            lower=np.array([45,30,45], dtype="uint8")
            higher=np.array([80,255,98], dtype="uint8")
            mask=cv2.inRange(color, lower, higher)
            
            #create an target?
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            for cnt in contours:
                if cv2.contourArea(cnt) > 500:
                    x, y, w, h = cv2.boundingRect(cnt)
                    cv2.rectangle(cv_image, (x, y), (x+w, y+h), (0,125,125), 2)


            
            self.publisher.publish(self.bridge.cv2_to_imgmsg(mask))
        
        except Exception as e:
        
            self.get_logger().error(f"Failed to convert image: {e}")
        
            return


        # Display image
        cv2.imshow("Drone Camera View", cv_image)
        cv2.imshow("Gate detection", mask)
        
        cv2.waitKey(1)  # required for OpenCV window to refresh
        #The physical drone driver creates it's own windows for the image_raw topic, so better 




def main(args=None):
    rclpy.init(args=args)

    node = TargetDetection()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()