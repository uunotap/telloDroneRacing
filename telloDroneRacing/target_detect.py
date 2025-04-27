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

        cv2.namedWindow("drone detection view", cv2.WINDOW_NORMAL)
        cv2.resizeWindow("drone detection view",1080, 540)
        cv2.moveWindow("drone detection view", 0, 0)
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

        # qr-code detection
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        self.aruco_parameters = cv2.aruco.DetectorParameters()
        self.aruco_detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.aruco_parameters)




        self.get_logger().info("Target_detection node has been started.")
        ##^^


    def listener_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            #Gate detection here?
            #Pub//Call planning?
        
            gray=cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
            #flipped=cv2.flip(gray,1)
        
            #Detect green gate?
            color=cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV) #change to a limited color scheme
            
            lower=np.array([45,30,45], dtype="uint8")
            higher=np.array([80,255,98], dtype="uint8")

            mask=cv2.inRange(color, lower, higher)

            #Detect red "gate"? -> stop sign?
            
            lower2=np.array([0,0,50], dtype="uint8")
            higher2=np.array([30,30,255], dtype="uint8")
            #I don't want to use that color shift it breaks red in to green
            mask2=cv2.inRange(cv_image, lower2, higher2)



            #detect 3(?) qr-codes create a mask
            corners, ids, _ = self.aruco_detector.detectMarkers(gray)

            
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


            if corners:
                
                markers=[]
                for p in corners:
                    qr_outline=p[0]
                    #center=(int(np.mean(qr_outline[:, 0])), int(np.mean(qr_outline[:, 1])))
                    
                    
                    center = np.mean(qr_outline, axis=0)
                    center_tuple = (int(center[0]), int(center[1]))

                    markers.append((center_tuple))
                    
                    cv2.circle(cv_image, center_tuple, 5, (0,0,255),-1)

                
                if len(markers)>1:
                    #AAAAAAAAAAAAAAAAA, why can't i get this to work
                    #[np.array(markers).reshape((-1,1,2)).astype(np.int32)]
                    #cv2.drawContours(cv_image, markers , 1,(75,125,125),0)
                    #hull=cv2.convexHull(markers).reshape(-1,1,2)
                    #area=cv2.contourArea(hull)


                    center = np.mean(markers, axis=0)
                    center_tuple = (int(center[0]), int(center[1]))
                    
                    cv2.circle(cv_image, center_tuple, 5, (0,0,255),-1)
                    gates.append((center_tuple, len(markers) * 225))

                else:
                    gates.append((markers[0],200))
        

            contoursR, _ = cv2.findContours(mask2, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
            for cnt in contoursR:
                if cv2.contourArea(cnt) > 500:
                    x, y, w, h = cv2.boundingRect(cnt)
                    cv2.rectangle(cv_image, (x, y), (x+w, y+h), (255,0,0), 2)
                    
                    center=(int(x+w/2),int(y+h/2))
                    
                    cv2.circle(cv_image, center,5,(0,255,0), -1)


                    
                    #self.get_logger().info("detection")
                    #lsit of contours? -> [(center off x-y, size), ...] choose the "largest"?
                    gates.append((center, -(w+h)))





            tar=Point()
            
            if gates != []:
                
                #self.get_logger().info("Found target!")
                if len(gates)>1:
                    #Adding "weak" center point
                    points=[g[0] for g in gates]
                    center = np.mean(points, axis=0)
                    center_tuple = (int(center[0]), int(center[1]))
                    cv2.circle(cv_image, center_tuple, 8, (125,125,125),-1)
                    gates.append((center_tuple, 449))
                




                closest = max(gates, key=lambda x: x[1]) #largst so probably nearby
                                                
                cv2.circle(cv_image, closest[0],10,(255,0,0), 0)
                self.imag_pub.publish(self.bridge.cv2_to_imgmsg(cv_image))    
                
                
                #self.get_logger().error(f"closest : {closest}")
                tar.x=float(closest[0][0])
                tar.y=float(closest[0][1])
                tar.z= float(closest[1])
            else:
                #self.get_logger().info("no target!")
                tar.z = -1.0 #Tell the controller there's no target
                
            self.target_pub.publish(tar)
            #Stop detection pass some value through?
            #jank way to gain the size
            #s= cv_image.shape
            #self.get_logger().error(f"shape: {s}")
            #self.get_logger().info(w)
            

        except Exception as e:
        
            self.get_logger().error(f"Image processing failed with the following error\n {e}")
        
            return


        # Display image
        #cv2.imshow("Drone Camera View", cv_image)
        
        #to see the mask
        #cv2.imshow("Gate detection", mask)
        
        #To all layers of processing
        cv2.imshow("drone detection view",np.hstack([cv2.cvtColor(mask2, cv2.COLOR_GRAY2BGR), cv_image]))


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