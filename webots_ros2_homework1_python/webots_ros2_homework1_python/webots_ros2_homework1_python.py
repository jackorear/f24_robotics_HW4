import rclpy
# import the ROS2 python libraries
from rclpy.node import Node
# import the Twist module from geometry_msgs interface
from geometry_msgs.msg import Twist
# import the LaserScan module from sensor_msgs interface
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from apriltag_ros_msgs.msg import AprilTagDetectionArray
# import Quality of Service library, to set the correct profile and reliability in order to read sensor data.
from rclpy.qos import ReliabilityPolicy, QoSProfile
import math
import time
import random

LINEAR_VEL = 0.15
ANGULAR_VEL = 1.0
STOP_DISTANCE = 0.4
LIDAR_ERROR = 0.05
WALL_DISTANCE = 0.35
LIDAR_AVOID_DISTANCE = .7
SAFE_STOP_DISTANCE = STOP_DISTANCE + LIDAR_ERROR
MAX_MOVE_DIST = 2
RIGHT_SIDE_INDEX = 270
RIGHT_FRONT_INDEX = 210
LEFT_FRONT_INDEX=150
LEFT_SIDE_INDEX=90

class RandomWalk(Node):

    def __init__(self):
        # Initialize the publisher
        super().__init__('random_walk_node')
        self.scan_cleaned = []
        self.stall = False
        self.turtlebot_moving = False
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.subscriber1 = self.create_subscription(
            LaserScan,
            '/scan',
            self.listener_callback1,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.subscriber2 = self.create_subscription(
            Odometry,
            '/odom',
            self.listener_callback2,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.apriltag_subscriber = self.create_subscription(
            AprilTagDetectionArray,
            '/apriltag_detections',
            self.apriltag_callback,
            10
        )
        self.laser_forward = 0
        self.odom_data = 0
        self.pose_saved = ''
        self.cmd = Twist()
        self.timer = self.create_timer(0.25, self.timer_callback)
        self.current_position = None
        self.orientation = 0.0
        self.start_x = 0.0
        self.start_y = 0.0
        self.distance_from_start = 0.0  # Track distance from start
        self.tag_detected = False
        self.tag_pose = None
        # Open a file for writing the position
        self.position_file = open('position.txt', 'w')

    def apriltag_callback(self, msg):
        """Callback for AprilTag detections."""
        if msg.detections:
            self.tag_detected = True
            self.tag_pose = msg.detections[0].pose.pose
            self.get_logger().info(f"AprilTag detected with ID: {msg.detections[0].id}")
        else:
            self.tag_detected = False
            self.tag_pose = None

    def dist_from_start(self, pos):
        """Calculate distance from the starting position."""
        (x, y) = pos
        dx = x - self.start_x
        dy = y - self.start_y
        return math.sqrt(dx**2 + dy**2)
        
    def quaternion_to_yaw(self, q):
        # Quaternion to Euler angles (yaw)
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        return yaw
        
    def normalize_angle(self, angle):
        """Normalize the angle to be within [-pi, pi]"""
        return math.atan2(math.sin(angle), math.cos(angle))
        
    def turn_x_deg(self, x):
        x = math.radians(x)
        turn_duration = (x / ANGULAR_VEL)
        self.cmd.angular.z = ANGULAR_VEL
        self.cmd.linear.x = 0.0
        self.publisher_.publish(self.cmd)
        time.sleep(turn_duration)
        self.cmd.angular.z = 0.0
        self.publisher_.publish(self.cmd)
        return
        
    def move_x_dist(self, x):
        if x > MAX_MOVE_DIST:
            x = MAX_MOVE_DIST
        elif x < STOP_DISTANCE:
            x = 0.0
        else:
            x = x - STOP_DISTANCE
        self.cmd.linear.x = LINEAR_VEL
        self.cmd.angular.z = 0.0
        self.publisher_.publish(self.cmd)
        self.turtlebot_moving = True
        time.sleep(x / LINEAR_VEL)
        self.cmd.linear.x = 0.0
        self.publisher_.publish(self.cmd)
        self.turtlebot_moving = False
        return
    
    def listener_callback1(self, msg1):
        scan = msg1.ranges
        self.scan_cleaned = [
            reading if reading != float('Inf') else 3.5 for reading in scan
        ]
            
    def listener_callback2(self, msg2):
        position = msg2.pose.pose.position
        self.current_position = (position.x, position.y)
        self.distance_from_start = self.dist_from_start(self.current_position)
        self.get_logger().info('Current position: {}'.format(self.current_position))
        self.get_logger().info('Distance from start: {}'.format(self.distance_from_start))
        if self.start_x == 0.0 and self.start_y == 0.0:
            self.start_x = position.x
            self.start_y = position.y
        self.position_file.write(f'{position.x}, {position.y}\n')
        self.position_file.flush()
           
    def timer_callback(self):
        if self.tag_detected and self.tag_pose:
            self.get_logger().info('AprilTag detected. Stopping to process the tag.')
            self.cmd.linear.x = 0.0
            self.cmd.angular.z = 0.0
            self.publisher_.publish(self.cmd)
            return
        
        if len(self.scan_cleaned) == 0:
            self.turtlebot_moving = False
            return
        
        left_lidar_min = min(self.scan_cleaned[LEFT_SIDE_INDEX:LEFT_FRONT_INDEX])
        right_lidar_min = min(self.scan_cleaned[RIGHT_FRONT_INDEX:RIGHT_SIDE_INDEX])
        front_lidar_min = min(self.scan_cleaned[LEFT_FRONT_INDEX:RIGHT_FRONT_INDEX])
        
        if front_lidar_min < SAFE_STOP_DISTANCE:
            self.cmd.linear.x = 0.0
            self.cmd.angular.z = ANGULAR_VEL * 0.5
        elif right_lidar_min < WALL_DISTANCE - LIDAR_ERROR:
            self.cmd.angular.z = ANGULAR_VEL * 0.5
            self.cmd.linear.x = LINEAR_VEL * 0.5
        elif right_lidar_min > WALL_DISTANCE + LIDAR_ERROR:
            self.cmd.angular.z = -ANGULAR_VEL * 0.5
            self.cmd.linear.x = LINEAR_VEL * 0.5
        else:
            random_angle = math.radians(random.uniform(-45, 45))
            self.cmd.angular.z = random_angle
            self.cmd.linear.x = LINEAR_VEL
        
        self.publisher_.publish(self.cmd)
        
def __del__(self):
        self.position_file.close()    
    
def main(args=None):
    rclpy.init(args=args)
    random_walk_node = RandomWalk()
    rclpy.spin(random_walk_node)
    random_walk_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
