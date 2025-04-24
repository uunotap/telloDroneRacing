import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from rclpy.qos import QoSProfile, QoSReliabilityPolicy


import cv2
from cv_bridge import CvBridge

class CamSubscriber(Node):

    def __init__(self):
        super().__init__('image_processor')
        
        self.bridge = CvBridge()


        #Tello drone/sim require naming a policy for connection
        qos_profile = QoSProfile(depth=10)
        qos_profile.reliability = QoSReliabilityPolicy.BEST_EFFORT
        
        self.subscription = self.create_subscription(
            Image,
            '/drone1/image_raw',
            self.listener_callback,
            qos_profile)
        
        self.get_logger().info("ImageProcessor node has been started.")



    def listener_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            #Gate detection here?
            #Pub for the planning to use
            #Or in another node which this one calls during an callback

        except Exception as e:
            
            self.get_logger().error(f"Failed to convert image: {e}")
            return


        # Display image
        cv2.imshow("Drone Camera View", cv_image)
        cv2.waitKey(1)  # required for OpenCV window to refresh
        #The physical drone driver creates it's own windows for the image_raw topic, so better 




def main(args=None):
    rclpy.init(args=args)

    node = CamSubscriber()

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