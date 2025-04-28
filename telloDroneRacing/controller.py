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
    LANDING = 3

class Controller(Node):

    def __init__(self):
        super().__init__('controller')

        self.bridge = CvBridge()
        #Tello drone/sim require naming a policy for connection
        qos_profile = QoSProfile(depth=10)
        qos_profile.reliability = QoSReliabilityPolicy.BEST_EFFORT
        
        
        #self.mov_pub = self.create_publisher(Twist,'/drone1/cmd_vel', 10)
        #self.serv_client=self.create_client(TelloAction, "/drone1/tello_action")
        
        self.mov_pub = self.create_publisher(Twist,'/cmd_vel', 10)
        self.serv_client=self.create_client(TelloAction, "/tello_action")

        #/cmd_vel
        #/drone1/cmd_vel
        #Type: geometry_msgs/msg/Twist
        
        #/tello_action
        #/drone1/tello_action


        # subscription to target, which is the offset from the center of the camera feed from the closest detected gate 
        self.subscription = self.create_subscription(Point, '/target', self.target_callback, qos_profile)
        

        

        while not self.serv_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /tello_action service...')
        
        self.state = State.SEEKING
        act=TelloAction.Request()
        act.cmd ="takeoff"
        self.serv_client.call_async(act)


        self.get_logger().info("controller has been started.")


        self.create_timer(0.1, self.send_cmd_vel)  # every 0.1s = 10Hz
        self.current_cmd = Twist()


    def send_cmd_vel(self):

        cmd=self.current_cmd
        minimpulse=0.05
        for field in ['x', 'y', 'z']:
            val = getattr(cmd.linear, field)
            if abs(val) < minimpulse and abs(val) > 0.0:  # Skip perfect zeros
                setattr(cmd.linear, field, minimpulse * (1 if val >= 0 else -1))

        for field in ['x', 'y', 'z']:
            val = getattr(cmd.angular, field)
            if abs(val) < minimpulse and abs(val) > 0.0:
                setattr(cmd.angular, field, minimpulse * (1 if val >= 0 else -1))
        
        if cmd.angular.z>0:
            if cmd.linear.x>0:
                cmd.linear.x+=0.05
            
        
        
        self.mov_pub.publish(cmd)


        ##^^


    def target_callback(self, msg):
        #self.get_logger().error(f"msg: {msg}")
        cmd=Twist()
        if int(msg.z) != -1:
            self.state = State.MOVING
        else:
            self.state = State.SEEKING
            cmd.angular.x = 0.0
            cmd.linear.x  = 0.0
            cmd.angular.y = 0.0
            cmd.linear.y  = 0.0
            cmd.angular.z = 0.0
            cmd.linear.z  = 0.0
            
        if self.state == State.SEEKING:
                
                self.get_logger().info("Seeking!")
                cmd.angular.z = -0.5
                #cmd.linear.x  =  0.0001
                cmd.linear.z  =  0.001  #hotfix3 get some height
                if int(msg.z)==-1:
                    self.state= State.RESET
        
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

                if abs(error_x)>20.0:
                    self.get_logger().info("Narrowing the x_offset!")
                    cmd.linear.z=0.0

                    cmd.linear.y = -error_x * 0.0005 #move diagonally

                    #cmd.linear.z = 0.005 #move a bit up #hotfix 1
                    
                    cmd.angular.z = -error_x * 0.0005 #turn

                    #cmd.linear.x = -0.00005 #move backwards
                    if int(msg.z) < 800:
                        self.get_logger().info("Going a bit closer!")
                        cmd.linear.x = 0.2
                        
                        
                        
                        #If there is definetly a red sign a head

                

                if abs(error_y)>10.0:
                    if error_y == -360: #due to shite hotfix 260<-360
                        self.get_logger().info("Y IS WRONG!")
                        
                        self.state = State.SEEKING
                    else: 
                        self.get_logger().info("Narrowing the y_offset!")
                    
                        cmd.linear.x = -error_y * 0.00005 #Dunno maybe good?
                        cmd.linear.z = -error_y * 0.0005
                    

                        
                else:
                    self.get_logger().info("Moving!")
                    cmd.linear.x = 0.75


        #self.get_logger().error(f"movement:\n {cmd}")
        self.current_cmd = cmd 

        if int(msg.z)<=-800:
            self.get_logger().info("Landing...")
            self.state=State.LANDING
            act=TelloAction.Request()
            act.cmd ="land"
            self.serv_client.call(act)






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