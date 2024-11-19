import rclpy
# import the ROS2 python libraries
from rclpy.node import Node
# import the Twist module from geometry_msgs interface
from geometry_msgs.msg import Twist
# import the LaserScan module from sensor_msgs interface
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
# import Quality of Service library, to set the correct profile and reliability in order to read sensor data.
from rclpy.qos import ReliabilityPolicy, QoSProfile
import math

LINEAR_VEL = 0.150
ROTATIONAL_VEL = math.radians(30)
DISTANCE = 5
ROTATION = math.radians(180)
MODE = 0
# 1 for distance, 0 for rotation

def quaternion_to_yaw(self, q):

        # Quaternion to Euler angles (yaw)

        siny_cosp = 2 * (q.w * q.z + q.x * q.y)

        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)

        yaw = math.atan2(siny_cosp, cosy_cosp)

        return yaw

class RandomWalk(Node):

    def __init__(self):
        # Initialize the publisher
        super().__init__('random_walk_node')
        self.scan_cleaned = []
        self.stall = False
        self.turtlebot_moving = True
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        #self.subscriber1 = self.create_subscription(
        #    LaserScan,
        #    '/scan',
        #    self.listener_callback1,
        #    QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.subscriber2 = self.create_subscription(
            Odometry,
            '/odom',
            self.listener_callback2,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.laser_forward = 0
        self.odom_data = 0
        timer_period = 0.1
        self.cmd = Twist()
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.timer_count = 0
        self.position_file = open('position.txt', 'a')
        self.position_start_write = False
        self.position_stop_write = False
        self.orientation_z = 0.0
    
    # subscribes to laser scan publishes
    #def listener_callback1(self, msg1):
        # self.get_logger().info('scan: "%s"' % msg1.ranges)
        #scan = msg1.ranges
        #self.scan_cleaned = []
       
        # self.get_logger().info('scan: "%s"' % scan)
        # self.get_logger().info('--------ENDOFSCAN--------')
        # Assume 360 range measurements
        #for reading in scan:
        #    if reading == float('Inf'):
        #        self.scan_cleaned.append(3.5)
        #    elif math.isnan(reading):
        #        self.scan_cleaned.append(0.0)
        #    else:
        #        self.scan_cleaned.append(reading)

    # subscribes to odometry publishes
    def listener_callback2(self, msg2):
        position = msg2.pose.pose.position
        orientation = msg2.pose.pose.orientation
        (posx, posy, posz) = (position.x, position.y, position.z)
        (qx, qy, qz, qw) = (orientation.x, orientation.y, orientation.z, orientation.w)

        self.orientation_z = quaternion_to_yaw(self, orientation)

        if MODE == 1:
            if self.position_start_write == False: 
                self.position_file.write(f'Speed: {LINEAR_VEL}\nDistance: {DISTANCE}\nStart: {position.x}, {position.y}\n')
                self.position_start_write = True
            
            elif self.turtlebot_moving == False and self.position_stop_write == False:
                self.position_file.write(f'Stop: {position.x}, {position.y}\n\n')
                self.position_stop_write = True
            
        else:
            if self.position_start_write == False: 
                self.position_file.write(f'Speed: {ROTATIONAL_VEL}\nRotation: {ROTATION}\nStart: {self.orientation_z}\n')
                self.position_start_write = True
            
            elif self.turtlebot_moving == False and self.position_stop_write == False:
                self.position_file.write(f'Stop: {self.orientation_z}\n\n')
                self.position_stop_write = True

        return None
        
    def timer_callback(self):

        self.timer_count = self.timer_count + 1
        # self.get_logger().info('Timer count: %s' % self.timer_count)
        
        if MODE == 1:
            if (LINEAR_VEL * self.timer_count * .1 >= DISTANCE):
                self.cmd.linear.x = 0.0
                self.turtlebot_moving = False
            else: 
                self.cmd.linear.x = LINEAR_VEL

        else:
            if (ROTATIONAL_VEL * self.timer_count * .1 >= ROTATION) and (self.timer_count != 1):
                self.cmd.angular.z = 0.0
                self.turtlebot_moving = False
                self.get_logger().info('1')
            else:
                self.cmd.angular.z = ROTATIONAL_VEL
                self.get_logger().info('2')
        
        self.publisher_.publish(self.cmd)
        ##self.get_logger().info('Z Velocity: %s' % self.cmd.angular.z)
        ##self.get_logger().info('Rotational vel: %s' % (ROTATIONAL_VEL * self.timer_count * .1))
        ##self.get_logger().info('Rotation ideal: %s' % ROTATION)
        self.get_logger().info('Orientation z: %s' % self.orientation_z)
        
        # Display the message on the console
        # self.get_logger().info('Publishing: "%s"' % self.cmd)

def main(args=None):
    # initialize the ROS communication
    rclpy.init(args=args)
    # declare the node constructor
    random_walk_node = RandomWalk()
    # pause the program execution, waits for a request to kill the node (ctrl+c)
    # https://answers.ros.org/question/388589/what-does-rclcppspin-actually-do-what-is-spin-node-mean/
    rclpy.spin(random_walk_node)
    # Explicity destroy the node
    random_walk_node.destroy_node()
    # shutdown the ROS communication
    rclpy.shutdown()



if __name__ == '__main__':
    main()