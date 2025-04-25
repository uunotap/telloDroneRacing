import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from rclpy.qos import QoSProfile, QoSReliabilityPolicy

import numpy as np
import cv2
from cv_bridge import CvBridge

from geometry_msgs.msg import  Point

class TargetDetection(Node):
    def __init__(self):
        super().__init__('gate_detector')
        
        self.bridge = CvBridge()


        #Tello drone/sim require naming a policy for connection
        qos_profile = QoSProfile(depth=10)
        qos_profile.reliability = QoSReliabilityPolicy.BEST_EFFORT
        
        self.imag_pub = self.create_publisher(Image,'/gates',qos_profile)
        self.target_pub = self.create_publisher(Point,'/target',qos_profile)

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


            #detect 3(?) qr-codes create a mask
            #cv2.findContours
        
            #Combine a mask of color and qr code contours?
            
            #create an target?    
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            gates=[]
            
            for cnt in contours:
                if cv2.contourArea(cnt) > 500:
                    x, y, w, h = cv2.boundingRect(cnt)
                    cv2.rectangle(cv_image, (x, y), (x+w, y+h), (0,125,125), 2)
                    center=(int(x+w/2),int(y+h/2))
                    cv2.circle(cv_image, center,5,(0,125,125), -1)
                    #self.get_logger().info("detection")
                    #lsit of contours? -> [(center off x-y, size), ...] choose the "largest"?
                    gates.append((center, w+h))
            
            tar=Point()
            if gates != []:
                closest = max(gates, key=lambda x: x[1]) #largst so probably nearby
                cv2.circle(cv_image, closest[0],10,(255,0,0), 0)
                self.imag_pub.publish(self.bridge.cv2_to_imgmsg(cv_image))    
                tar.x=float(closest[0][0])
                tar.y=float(closest[0][1])
                

            else:
                tar.z = -1.0 #Tell the controller there's no target
                
            #Stop detection pass some value through?


            self.target_pub.publish(tar)
            #jank way to gain the size
            #s= cv_image.shape
            #self.get_logger().error(f"shape: {s}")
            #self.get_logger().info(w)
            

        except Exception as e:
        
            self.get_logger().error(f"Failed to convert image: {e}")
        
            return


        # Display image
        cv2.imshow("Drone Camera View", cv_image)
        
        #to see the mask
        #cv2.imshow("Gate detection", mask)
        
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