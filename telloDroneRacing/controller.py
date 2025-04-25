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
import enum
class State(enum.Enum):
    RESET = 0
    SEEKING = 1
    MOVING = 2

class Controller(Node):

    def __init__(self):
        super().__init__('controller')

        self.bridge = CvBridge()
        #Tello drone/sim require naming a policy for connection
        qos_profile = QoSProfile(depth=10)
        qos_profile.reliability = QoSReliabilityPolicy.BEST_EFFORT
        
        
        self.mov_pub = self.create_publisher(Twist,'/drone1/cmd_vel', 10)
        #/cmd_vel
        #/drone1/cmd_vel
        #Type: geometry_msgs/msg/Twist
        

        # subscription to target, which is the offset from the center of the camera feed from the closest detected gate 
        self.subscription = self.create_subscription(Point, '/target', self.target_callback, qos_profile)
        

        

        self.serv_client=self.create_client(TelloAction, "drone1/tello_action")
        while not self.serv_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /drone1/tello_action service...')
        
        self.state = State.SEEKING
        act=TelloAction.Request()
        act.cmd ="takeoff"
        self.serv_client.call_async(act)


        self.get_logger().info("controller has been started.")
        ##^^


    def target_callback(self, msg):
        self.get_logger().error(f"msg: {msg}")
        cmd=Twist()
        if msg.z != -1.0:
            self.state == State.SEEKING
        else:
            self.state == State.MOVING

        try:
            if self.state == State.RESET:
                cmd.angular.x = 0.0
                cmd.linear.x  = 0.0
                cmd.angular.y = 0.0
                cmd.linear.y  = 0.0
                cmd.angular.z = 0.0
                cmd.linear.z  = 0.0
                self.state=State.SEEKING
                self.get_logger().info("--RESET--")

            elif self.state == State.SEEKING:
                if float(msg.z) ==-1.0:
                    self.get_logger().info("Seeking!")
                    cmd.angular.z = -0.2
                    cmd.linear.x  =  0.0  
                else:
                    self.get_logger().info("HElvetti")    
                    self.state= State.MOVING
        
            if self.state == State.MOVING:                    
                self.get_logger().info("Doing")
                

                #image size, refer to jank code in target detect
                #(720, 960, 3)
                # x center = 480            
                # y center = 360

                #lower x is left, higher x is right 
                x_offset=msg.x

                #lower y is up, higher y is down
                y_offset=msg.y

                error_x = x_offset - 480 # how much to move
                error_y = y_offset - 360 # how much to move


                

                if abs(error_y)>5.0:
                    self.get_logger().info("Narrowing the y_offset!")
                    #adjust height
   
                    cmd.linear.z = -error_y * 0.0005
                    #adjust relation
                elif abs(error_x)>5.0:
                    self.get_logger().info("Narrowing the x_offset!")
                    cmd.linear.z=0.0
                    cmd.linear.y = -error_x * 0.0005 
                    cmd.angular.z = -error_x * 0.00005
                    cmd.linear.x = -0.2
                else:
                    self.get_logger().info("Moving!")
                    cmd.linear.x = 0.2

        except Exception as e:
        
            self.get_logger().error(f"Failed to convert image: {e}")
        
            return            
                #cmd.angular.x = -error_x * 0.0005  #turn?

                #  # adjust for horizontal error
                #cmd.linear.x  = 0 # move worward  

        finally:
            self.mov_pub.publish(cmd)





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


if __name__ == '__main__':
    main()