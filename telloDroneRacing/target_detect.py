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

        #cv2.namedWindow("masks", cv2.WINDOW_NORMAL)
        #cv2.resizeWindow("masks",900, 300)
        


        #Tello drone/sim require naming a policy for connection
        qos_profile = QoSProfile(depth=10)
        qos_profile.reliability = QoSReliabilityPolicy.BEST_EFFORT
        
        self.imag_pub = self.create_publisher(Image,'/gates',qos_profile)
        self.target_pub = self.create_publisher(Point,'/target',qos_profile)
        
        
        
        ##Same as image_proc, which might not be needed on the physical drone

        #CHANGE
        self.subscription = self.create_subscription(Image,'/image_raw', self.listener_callback, qos_profile)
        #self.subscription = self.create_subscription(Image,'/drone1/image_raw', self.listener_callback, qos_profile)
        
	## TO SEE THE ACCEPTABLE ERROR RADIUS
        self.subscription = self.create_subscription(Point, '/err', self.update_err, qos_profile)
        
        self.x_err=0
        self.y_err=0 
        
        #'/drone1/image_raw'
        #'/image_raw'

        # qr-code detection
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        self.aruco_parameters = cv2.aruco.DetectorParameters()
        self.aruco_detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.aruco_parameters)




        self.get_logger().info("Target_detection node has been started.")
        ##^^


    def update_err(self, msg):
        #self.get_logger().error(f"err: {msg}")
        self.x_err=msg.x
        self.y_err=msg.y
        


    def listener_callback(self, msg):
	## TRY, to process image since the message can be unreliable(?). It is best practice(?)
        
        try:
            # The base image is converted to the cv2 usable format//python typing whatever
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            
            ## A couple different masks for differing detection stuff
            # For qr code detection, the gray colorspace should make for easier detection, if the library/method doesn't already limit the possible inputs.
            gray=cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
            # The green is a bit weird in this colors space (Especially when looking at red compared to it) but the mask seems to work pretty well.
            color=cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV) #change to a limited color scheme
            #* The gate tends to be a bit of a reddish green with this color range
            
            

            # Set font, for reading out the "sizes", or weights, of targets.
            font = cv2.FONT_HERSHEY_SIMPLEX
            font_scale = 0.7
            font_color = (0, 0, 0)
            thickness = 2






            # (blue green red) for whatever reason ?
            
            ## Mask for green, the color space might be a bit shite. 
            #But it limits the noise from the real camera.(?) Might not but I feel like it does :D
            lower=np.array([40,60,45], dtype="uint8")
            higher=np.array([95,255,130], dtype="uint8")

            mask_green=cv2.inRange(color, lower, higher)


            ## Mask for red
            #Detect red "gate"? -> stop sign?
            lower2=np.array([30,30,90], dtype="uint8")
            higher2=np.array([75,75,255], dtype="uint8")
            #I don't want to use that color shift it breaks red in to green
            mask_red=cv2.inRange(cv_image, lower2, higher2)



            #detect 3(?) qr-codes create a mask
            corners, ids, _ = self.aruco_detector.detectMarkers(gray)

            
            #Green target detection
            contours, _ = cv2.findContours(mask_green, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

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
                    
                    

            markers=[]
            ## If there are qr-codes
            if corners:
                
		## For each "corner, aka qr-code. Why didn't I just go with that? Because they are corners of a triangle haha, pointless abstraction...
                for p in corners:
                    qr_outline=p[0]
                    #center=(int(np.mean(qr_outline[:, 0])), int(np.mean(qr_outline[:, 1])))
                    
                    center = np.mean(qr_outline, axis=0)
                    center_tuple = (int(center[0]), int(center[1]))

                    markers.append((center_tuple))
                    ## Mark the qr-code location
                    cv2.circle(cv_image, center_tuple, 5, (0,0,255),-1)

                
                if len(markers)==3:
                    ## Better area calculation if it can be made to work.
                    #AAAAAAAAAAAAAAAAA, why can't i get this to work
                    #[np.array(markers).reshape((-1,1,2)).astype(np.int32)]
                    #cv2.drawContours(cv_image, markers , 1,(75,125,125),0)
                    #hull=cv2.convexHull(markers).reshape(-1,1,2)
                    #area=cv2.contourArea(hull)
		
			## THE AVERAGE COORDINATES OF THE QR-CODES, it is the "center". This does work suprisingly well.
                    center = np.mean(markers, axis=0)
                    center_tuple = (int(center[0]), int(center[1])-50)
                    
                    ### MAKING A CENTRAL POINT BETWEEN QR CODES WHICH HAS A LARGE VALUE
                    ## This might need to be adjusted to be larger than the average weak center, in all cases
                    cv2.circle(cv_image, center_tuple, 5, (0,0,255),-1)
                    gates.append((center_tuple, len(markers) * 260))
                
                if len(markers)==2:
                    ## Better area calculation if it can be made to work.
                    #AAAAAAAAAAAAAAAAA, why can't i get this to work
                    #[np.array(markers).reshape((-1,1,2)).astype(np.int32)]
                    #cv2.drawContours(cv_image, markers , 1,(75,125,125),0)
                    #hull=cv2.convexHull(markers).reshape(-1,1,2)
                    #area=cv2.contourArea(hull)
		
			## THE AVERAGE COORDINATES OF THE QR-CODES, it is the "center". This does work suprisingly well.
                    center = np.mean(markers, axis=0)
                    center_tuple = (int(center[0]), int(center[1])-200)
                    
                    ### MAKING A CENTRAL POINT BETWEEN QR CODES WHICH HAS A LARGE VALUE
                    ## This might need to be adjusted to be larger than the average weak center, in all cases
                    cv2.circle(cv_image, center_tuple, 5, (0,0,255),-1)
                    gates.append((center_tuple, len(markers) * 260))

                else:
                ## IF there aren't multiple qr-codes it should have a weak value
                    gates.append((markers[0],230))
        

            ## FIND RED "AREAS", though this might conflict with the "weak" central point thingamajig.
            ## Might be good to but this after the central point creation?
            contoursR, _ = cv2.findContours(mask_red, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            for cnt in contoursR:
                if cv2.contourArea(cnt) > 500:
                    x, y, w, h = cv2.boundingRect(cnt)
                    cv2.rectangle(cv_image, (x, y), (x+w, y+h), (255,0,0), 2)
                    
                    center=(int(x+w/2),int(y+h/2))
                    
                    cv2.circle(cv_image, center,5,(0,255,0), -1)

     
                    #self.get_logger().info("detection")
                    #lsit of contours? -> [(center off x-y, size), ...] choose the "largest"?
                    
                    ## The weight is negative since it shouldn't be focused compared to green.

                    gates.append((center, -(w+h)))




            ## Point has X, Y, Z. Using the X and Y to communicate relative distance using the image. 
            ## Z for communicating the size/"distance" of the gate
            # Create message before, checking if gates are real.. since the -1 communicates no detection. 
            tar=Point()
            
            
            if gates != []:
                
                #self.get_logger().info("Found target!")
                if len(gates)>1:
                    ## Adding "weak" center point, if there are multiple detections.
                    points=[g[0] for g in gates]
                    center = np.mean(points, axis=0)
                    center_tuple = (int(center[0]), int(center[1])-50)
                    cv2.circle(cv_image, center_tuple, 8, (125,125,125),-1)
                    gates.append((center_tuple, 450)) #hotfix 4 600<-500<-449, might be too high now.
                

                
                ## Additional gate processing
                for g in gates:
                    ## ADDITIONAL PROCESSING
                    g=((g[0][0],g[0][1]-50),g[1]) #hotfix2, adjust height a bit up

                    ## WRITING DEBUG "INFO", weight//size whatever, TO EACH DETECTED TARGET
                        
                        # Get the text size to center it nicely
                    (text_width, text_height), _ = cv2.getTextSize(str(g[1]), font, font_scale, thickness)

                    # Calculate text position slightly under the point
                    text_x = g[0][0] - text_width // 2
                    text_y = g[0][1] + 20 + text_height // 2  # 20 pixels below the point

                    # Write the text
                    cv2.putText(cv_image, str(g[1]), (text_x, text_y), font, font_scale, font_color, thickness)        



		#-->
                closest = max(gates, key=lambda x: x[1]) #largst so probably nearby                                 
                cv2.circle(cv_image, closest[0],10,(255,0,0), 0)
                
                
                self.imag_pub.publish(self.bridge.cv2_to_imgmsg(cv_image))    
                

		## Passing the "X" and "Y" of the detected target. 
		#It is in relation to the image that the drone feeds in to the detection... yes.
                
                #self.get_logger().error(f"closest : {closest}")
                tar.x=float(closest[0][0])
                tar.y=float(closest[0][1])
                tar.z= float(closest[1])


            else:
                # Telling the controller there is no target in detection.
                tar.z = -1.0 #Tell the controller there's no target
                #self.get_logger().info("no target!")
            self.target_pub.publish(tar)
            
            #Stop detection pass some value through?
            # CURRENTLY, neagtive values other than -1 are the size of the red sign
            #* jank way to gain the size
            
            ## DEBUGGING 
            #s= cv_image.shape
            #self.get_logger().error(f"shape: {s}")
            #self.get_logger().info(w)
            

        except Exception as e:
        
            self.get_logger().error(f"Image processing failed with the following error\n {e}")
        
            return


        cv2.ellipse(cv_image, (480, 360), (int(self.x_err*2+1) ,int(self.y_err*2+1)), 0, 0, 360,(255,50,50),10, lineType=cv2.LINE_AA)
	
    ### MASK DEBUGGING	
        ## Making a combined mask of detections
        
        ## Green detection
        _, mask1= cv2.threshold(mask_green, 127,255,cv2.THRESH_BINARY)
        ## Red detection
        _, mask2= cv2.threshold(mask_red, 127,255,cv2.THRESH_BINARY)
        ##Blue, for qr-tag detection
        blue =np.zeros_like(mask1)
        for tag in markers:
            cv2.circle(blue, tag, 5,(255),-1)        
        
        detection=cv2.merge([blue, mask1, mask2])
        
        
        ## DEBUG To see the actualy color values that the mask is working with
        #cv2.imshow("masks",np.hstack([cv2.cvtColor(mask1, cv2.COLOR_GRAY2BGR),color,cv2.cvtColor(mask2, cv2.COLOR_GRAY2BGR)]))


	# Better to make an
        cv2.imshow("drone detection view",np.hstack([cv_image, detection]))

        
        cv2.waitKey(1)  # required for OpenCV window to refresh

def detect(self,c):
	shape ="null"
	peri = cv2.archLength(c, True)
	approx = cv2.approxPolyDP(c, 0.04 * peri, True)
	
	if len(approx) == 4
		(x,y,w,h) ) = cv2.boundingRect(approx)
		ar = w /float(h)
		shape = "square"

	else:
		shape = "circle"
	

return shape

	
	




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
