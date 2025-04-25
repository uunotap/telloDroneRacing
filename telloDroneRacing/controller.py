import rclpy
from rclpy.node import Node

from rclpy.qos import QoSProfile, QoSReliabilityPolicy


import cv2
from cv_bridge import CvBridge
import json


from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist, Point
from tello_msgs.srv import TelloAction

from std_msgs.msg import String
from std_msgs.msg import Int16




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
        

        # subscription to target, which is the offset from the center of the camera feed from the closest detected gate 
        # 
        self.subscription = self.create_subscription(Point, '/target', self.target_callback, qos_profile)
        




        self.serv_client=self.create_client(TelloAction, "drone1/tello_action")
        while not self.serv_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /drone1/tello_action service...')

        act=TelloAction.Request()
        act.cmd ="takeoff"
        self.serv_client.call_async(act)


        self.get_logger().info("controller has been started.")
        ##^^


    def target_callback(self, msg):
        try:
            self.get_logger().info("Moving")
            #lower x is left higher right
            x_offset=msg.x
            #lower y is up higher is down
            y_offset=msg.y

            #image size, refer to jank code in target detect
            #(720, 960, 3)
            # x center = 480            
            # y center = 360

            cmd=Twist()

            error_x = x_offset - 480 # how much to turn

            cmd.linear.x = 0.2  # constant forward speed
            cmd.linear.y = -error_x * 0.005  # adjust for horizontal error

            self.mov_pub.publish(cmd)

            

        except Exception as e:
        
            self.get_logger().error(f"Failed to get target: {e}")
        
            return





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