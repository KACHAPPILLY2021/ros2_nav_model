import rclpy 
from rclpy.node import Node
from geometry_msgs.msg import Twist            
from sensor_msgs.msg import LaserScan    
from std_msgs.msg import Float64MultiArray
from rclpy.qos import qos_profile_sensor_data 

 
class Controller(Node):
  """
  Create a Controller class, which is a subclass of the Node 
  class for ROS2.
  """
  def __init__(self):
    """
    Class constructor to set up the node
    """
    ##################### ROS SETUP ####################################################
    # Initiate the Node class's constructor and give it a name
    super().__init__('Controller')
 
    # Create a subscriber
    # This node subscribes to messages of type Float64MultiArray  
    # over a topic named: /state_est
    # The message represents the current estimated state:
    #   [x, y, yaw]
    # The callback function is called as soon as a message 
    # is received.
    # The maximum number of queued messages is 10.
    self.subscription = self.create_subscription(
                        Float64MultiArray,
                        '/state_est',
                        self.state_estimate_callback,
                        10)
    self.subscription  # prevent unused variable warning
 
    # Create a subscriber
    # This node subscribes to messages of type 
    # sensor_msgs/LaserScan     
    self.scan_subscriber = self.create_subscription(
                           LaserScan,
                           '/laser/out',
                           self.scan_callback,
                           qos_profile=qos_profile_sensor_data)
                            
    # Create a publisher
    # This node publishes the desired linear and angular velocity of the robot (in the
    # robot chassis coordinate frame) to the /cmd_vel topic. Using the diff_drive
    # plugin enables the robot model to read this /cmd_vel topic and execute
    # the motion accordingly.
    self.publisher_ = self.create_publisher(
                      Twist, 
                      '/cmd_vel', 
                      10)
 
    # Initialize the LaserScan sensor readings to some large value
    # Values are in meters.
    self.leftfront_dist = 999999.9 # Left-front
    self.front_dist = 999999.9 # Front
    self.rightfront_dist = 999999.9 # Right-front
 
    ################### ROBOT CONTROL PARAMETERS ##################
    # Maximum forward speed of the robot in meters per second
    # Any faster than this and the robot risks falling over.
    self.forward_speed = 0.15
 
    # Current position and orientation of the robot in the global 
    # reference frame
    self.current_x = 0.0
    self.current_y = 0.0
    self.current_yaw = 0.0
 
    ############# obstacle FOLLOWING PARAMETERS #######################     
    # Finite states for the obstacle following mode
    #  "search for obstacle": Robot tries to locate the obstacle        
    #  "obstacle ahead": Robot turns away from the obstacle
    self.obstacle_following_state = "search for obstacle"
         
    # Set turning speeds (to the left) in rad/s 
    self.turning_speed_wf_fast = 3.5 

 
    # obstacle following distance threshold.
    # We want to try to keep within this distance from the obstacle.
    self.dist_thresh_wf = 2.50 # in meters  
 
  def state_estimate_callback(self, msg):
    """Extract the position and orientation data. 

    Args:
        msg (_type_): Important position values in decimals 
    """    

    # Update the current estimated state in the global reference frame
    curr_state = msg.data
    self.current_x = curr_state[0]
    self.current_y = curr_state[1]
    self.current_yaw = curr_state[2]
    
    self.follow_obstacle()
 
  def scan_callback(self, msg):
    """Method called everytime a Laser scan is published to
      /laser/out

    Args:
        msg (_type_): Read the laser scan data that indicates distances
                      to obstacles (e.g. obstacle) in meters and extract
                      5 distinct laser readings to work with.
                      Each reading is separated by 45 degrees.
                      Assumes 181 laser readings, separated by 1 degree. 
                      (e.g. -90 degrees to 90 degrees....0 to 180 degrees)
    """    
 
    #number_of_laser_beams = str(len(msg.ranges))       
    self.left_dist = msg.ranges[180]
    self.leftfront_dist = msg.ranges[135]
    self.front_dist = msg.ranges[90]
    self.rightfront_dist = msg.ranges[45]
    self.right_dist = msg.ranges[0]
             
  def follow_obstacle(self):
    """
    This method causes the robot to follow the boundary of a obstacle.
    """
    # Create a geometry_msgs/Twist message
    msg = Twist()
    msg.linear.x = 0.0
    msg.linear.y = 0.0
    msg.linear.z = 0.0
    msg.angular.x = 0.0
    msg.angular.y = 0.0
    msg.angular.z = 0.0        
 
    # Logic for following the obstacle
    # >d means no obstacle detected by that laser beam
    # <d means an obstacle was detected by that laser beam
    d = self.dist_thresh_wf

    if self.current_x>11 :
      self.obstacle_following_state = "STOPPING"
      msg.linear.x = 0.0
      msg.angular.z = 0.0
    elif self.leftfront_dist > d and self.front_dist > d and self.rightfront_dist > d:
      self.obstacle_following_state = "search for obstacle"
      msg.linear.x = self.forward_speed
    else :
      self.obstacle_following_state = "obstacle ahead"
      msg.angular.z = -self.turning_speed_wf_fast

    self.get_logger().info(self.obstacle_following_state)
    # Send velocity command to the robot
    self.publisher_.publish(msg)    
 
def main(args=None):
 
    # Initialize rclpy library
    rclpy.init(args=args)
     
    # Create the node
    controller = Controller()

    rclpy.spin(controller)

    controller.destroy_node()
    rclpy.shutdown()
 
if __name__ == '__main__':
    main()