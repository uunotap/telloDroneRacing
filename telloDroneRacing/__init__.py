#Required file for the general process
#...List of topics for ease of access, why not here?


###Connecting to the teleop simulation
##... from drone_racing_ros2 README
#    cd ~/drone_racing_ros2_ws
#    source install/setup.bash
#    export GAZEBO_MODEL_PATH=${PWD}/install/tello_gazebo/share/tello_gazebo/models
#    source /usr/share/gazebo/setup.sh
#    ros2 launch tello_gazebo simple_launch.py


## List of all(?) topics for simulation
#/drone1/camera_info
#Type: sensor_msgs/msg/CameraInfo
#/drone1/cmd_vel
#Type: geometry_msgs/msg/Twist
#/drone1/flight_data
#Type: tello_msgs/msg/FlightData
#/drone1/image_raw
#Type: sensor_msgs/msg/Image
#/drone1/joy
#Type: sensor_msgs/msg/Joy
#/drone1/joy/set_feedback
#Type: sensor_msgs/msg/JoyFeedback
#/drone1/odom
#Type: nav_msgs/msg/Odometry
#/drone1/tello_response
#Type: tello_msgs/msg/TelloResponse


###Connecting to physical drone
#cd ~/drone_racing_ros2_ws/
#colcon build
#source install/setup.bash
#ros2 launch tello_driver teleop_launch.py 

##list of all topics
#/camera_info
#Type: sensor_msgs/msg/CameraInfo
#/cmd_vel
#Type: geometry_msgs/msg/Twist
#/flight_data
#Type: tello_msgs/msg/FlightData
#/image_raw
#Type: sensor_msgs/msg/Image
#/joy
#Type: sensor_msgs/msg/Joy
#/joy/set_feedback
#Type: sensor_msgs/msg/JoyFeedback
#/parameter_events
#Type: rcl_interfaces/msg/ParameterEvent
#/rosout
#Type: rcl_interfaces/msg/Log
#/tello_response
#Type: tello_msgs/msg/TelloResponse


