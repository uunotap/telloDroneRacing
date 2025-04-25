import rclpy
from rclpy.node import Node

from rclpy.qos import QoSProfile, QoSReliabilityPolicy


import cv2
from cv_bridge import CvBridge
import json


from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from tello_msgs.srv import TelloAction

from std_msgs.msg import String

class Controller(Node):

    def __init__(self):
        super().__init__('gate_detector')

        self.bridge = CvBridge()
        #Tello drone/sim require naming a policy for connection
        qos_profile = QoSProfile(depth=10)
        qos_profile.reliability = QoSReliabilityPolicy.BEST_EFFORT
        
        
        self.mov_pub = self.create_publisher(Twist,'/drone1/cmd_vel', 10)
        #/cmd_vel
        #/drone1/cmd_vel
        #Type: geometry_msgs/msg/Twist
        

        ##Same as image_proc, which might not be needed on the physical drone
        self.subscription = self.create_subscription(
            Image,
            '/gates',
            self.image_callback,
            qos_profile)
        #'/drone1/image_raw'
        #'/image_raw'




        self.serv_client=self.create_client(TelloAction, "drone1/tello_action")
        while not self.serv_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /drone1/tello_action service...')

        act=TelloAction.Request()
        act.cmd ="takeoff"
        self.serv_client.call_async(act)


        self.get_logger().info("controller has been started.")
        ##^^


    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')



        except Exception as e:
        
            self.get_logger().error(f"Failed to convert image: {e}")
        
            return


        # Display image
        cv2.imshow("Drone Camera View", cv_image)
        cv2.waitKey(1)  # required for OpenCV window to refresh
        #The physical drone driver creates it's own windows for the image_raw topic, so better 




def main(args=None):

    rclpy.init(args=args)
    node = Controller()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        act=TelloAction.Request()
        act.cmd ="land"
        node.serv_client.call(act)

        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()