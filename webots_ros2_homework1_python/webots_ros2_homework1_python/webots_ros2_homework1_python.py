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
import time
import statistics

last_change_direction_time = time.time()

LINEAR_VEL = 0.18
ROT_VEL = 0.15
STOP_DISTANCE = 0.3
LIDAR_ERROR = 0.05
LIDAR_AVOID_DISTANCE = 0.6
SAFE_STOP_DISTANCE = STOP_DISTANCE + LIDAR_ERROR
LIDAR_WALL_SHORT_DISTANCE = 0.4
LIDAR_WALL_FAR_DISTANCE = 0.7
#RIGHT_SIDE_INDEX = 270
#RIGHT_FRONT_INDEX = 210 
#LEFT_FRONT_INDEX= 150
#LEFT_SIDE_INDEX= 90

RIGHT_SIDE_INDEX = 90
RIGHT_FRONT_INDEX = 30
LEFT_FRONT_INDEX = 330
LEFT_SIDE_INDEX = 270

# defines the linear speed of the robot based on how far away from the right wall it is
def linear_speed_calc(neg, distance, limit):
    return ((neg) * 10.0) * (distance - limit) # i didn't copy this from anything i just messed around in desmos till i found a curve i liked i know its scuffed i'm sorry

class RandomWalk(Node):

    def __init__(self):
        # Initialize the publisher
        super().__init__('random_walk_node')
        self.scan_cleaned = []
        self.stall = False
        self.stall_count = 0
        self.stall_rotating = False
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
        self.laser_forward = 0
        self.odom_data = 0
        timer_period = 0.5

        # Coordinates saved in order to calculate stalls
        self.pose_saved=''
        self.orien_saved=''
        self.front_distance_saved=0.0
        self.front_distance_saved_prev=0.0

        self.cmd = Twist()
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.found_wall = False
        self.rotating = False
        # False is start, True is following a wall

    
    # subscribes to laser scan publishes
    def listener_callback1(self, msg1):
        # self.get_logger().info('scan: "%s"' % msg1.ranges)
        scan = msg1.ranges
        self.scan_cleaned = []
       
        # self.get_logger().info('scan: "%s"' % scan)
        # self.get_logger().info('--------ENDOFSCAN--------')
        # Assume 360 range measurements
        for reading in scan:
            if reading == float('Inf'):
                self.scan_cleaned.append(3.5)
            elif math.isnan(reading):
                self.scan_cleaned.append(0.01)
            elif reading == float(0):
                self.scan_cleaned.append(10.0)
            else:
                self.scan_cleaned.append(reading)

    # subscribes to odometry publishes
    def listener_callback2(self, msg2):
        position = msg2.pose.pose.position
        orientation = msg2.pose.pose.orientation
        (posx, posy, posz) = (position.x, position.y, position.z)
        (qx, qy, qz, qw) = (orientation.x, orientation.y, orientation.z, orientation.w)
        

        # self.get_logger().info('self position: {},{},{}'.format(posx,posy,posz))
        # similarly for twist message if you need
        #Example of how to identify a stall..need better tuned position deltas; wheels spin and example fast
        #diffX = math.fabs(self.pose_saved.x- position.x)
        #diffY = math.fabs(self.pose_saved.y - position.y)
        #if (diffX < 0.0001 and diffY < 0.0001):
           #self.stall = True
        #else:
           #self.stall = False
        ## ----
        # fabs() returns the value as a float
        # not sure yet how pose_saved and position are different if pose_saved is defined by position
        if self.pose_saved == '':
            self.pose_saved = position
            self.orien_saved = orientation
            self.front_distance_saved_prev = self.front_distance_saved
        else: 
            if self.stall_count >= 40:
                diffX = math.fabs(self.pose_saved.x - position.x)
                diffY = math.fabs(self.pose_saved.y - position.y)
                diffZ = math.fabs(self.orien_saved.z - orientation.z)
                diffDist = math.fabs(self.front_distance_saved_prev - self.front_distance_saved)
                if ((diffX < 0.001 and diffY < 0.001 and diffZ < 0.001) or diffDist < 0.001): 
                    self.stall = True
                else: 
                    self.stall = False
                self.stall_count = 0
                self.pose_saved = position
                self.orien_saved = orientation
                self.front_distance_saved_prev = self.front_distance_saved
            else:
                self.stall_count = self.stall_count + 1


        #self.get_logger().info('diffX : %f ' % diffX)
        #self.get_logger().info('diffY : %f ' % diffY)
        #self.get_logger().info('diffZ : %f ' % diffZ)

        ## ----
           
        return None
        
    def timer_callback(self):
        if (len(self.scan_cleaned)==0):
            self.turtlebot_moving = False
            return
            
        #left_lidar_samples = self.scan_cleaned[LEFT_SIDE_INDEX:LEFT_FRONT_INDEX]
        #right_lidar_samples = self.scan_cleaned[RIGHT_FRONT_INDEX:RIGHT_SIDE_INDEX]
        #front_lidar_samples = self.scan_cleaned[LEFT_FRONT_INDEX:RIGHT_FRONT_INDEX]
        
        left_lidar_min  = min(self.scan_cleaned[LEFT_SIDE_INDEX:LEFT_FRONT_INDEX])
        right_lidar_min = min(self.scan_cleaned[RIGHT_FRONT_INDEX:RIGHT_SIDE_INDEX])
        front_lidar_min = min(self.scan_cleaned[LEFT_FRONT_INDEX:359] + self.scan_cleaned[0:RIGHT_FRONT_INDEX])
        self.front_distance_saved = left_lidar_min
        
        
        #right_lidar_avg = statistics.mean(self.scan_cleaned[RIGHT_FRONT_INDEX:RIGHT_SIDE_INDEX])
        #right_lidar_perp = min(self.scan_cleaned[230:250])

        #self.get_logger().info('left scan slice: "%s"'%  min(left_lidar_samples))
        #self.get_logger().info('front scan slice: "%s"'%  min(front_lidar_samples))
        #self.get_logger().info('right scan slice: "%s"'%  min(right_lidar_samples))

        # if the front lidar is less than the safe stopping distance (less than avoid distance), 
        # stops the robot's movement, publishes the speeds, and sets the moving flag to false

        if self.found_wall == False: # robot is starting/lost and does not know where a wall is
            if (front_lidar_min > LIDAR_WALL_SHORT_DISTANCE) & (right_lidar_min > LIDAR_WALL_SHORT_DISTANCE) & (left_lidar_min > LIDAR_WALL_SHORT_DISTANCE):
                self.cmd.linear.x = LINEAR_VEL
                self.cmd.angular.z = 0.0
                self.publisher_.publish(self.cmd)
                self.turtlebot_moving = True
                self.get_logger().info('Finding a wall.')
            else:
                # found a wall, rotating so that the wall is on the robot's right
                if (right_lidar_min > LIDAR_WALL_SHORT_DISTANCE):
                    self.cmd.linear.x = 0.0
                    self.cmd.angular.z = ROT_VEL
                    self.publisher_.publish(self.cmd)
                    self.turtlebot_moving = True
                    self.get_logger().info('Rotating until wall is on right.')
                else: 
                    self.found_wall = True
                    self.cmd.linear.x = 0.0
                    self.cmd.angular.z = 0.0
                    self.publisher_.publish(self.cmd)
                    self.turtlebot_moving = True
                    self.get_logger().info('Wall found and oriented. Proceeding to navigation.')

        else: # the robot has found a wall and is beginning normal navigation

            if (front_lidar_min < SAFE_STOP_DISTANCE): # if the robot is an unsafe distance away from an obstacle
                if self.turtlebot_moving == True:
                    self.cmd.linear.x = 0.0
                    self.cmd.angular.z = 0.0 
                    self.publisher_.publish(self.cmd)
                    self.turtlebot_moving = False   
                    self.stall_count = 0
                    self.stall_rotating = True
                    self.get_logger().info('Stopping due to unsafe distance from obstacle.')
                    return
            
            elif (self.rotating == True): # if the robot has been told to rotate for any reason
                                          # forces the robot to go to a right angle rather than continuing forward the moment there is any free area in front
                if front_lidar_min > (right_lidar_min + LIDAR_ERROR) & (front_lidar_min > LIDAR_AVOID_DISTANCE) & (right_lidar_min > LIDAR_AVOID_DISTANCE):
                    self.cmd.linear.x = 0.0
                    self.cmd.angular.z = 0.0
                    self.rotating = False
                    self.publisher_.publish(self.cmd)
                    self.turtlebot_moving = False
                    self.stall_count = 0
                    self.stall_rotating = True
                    self.get_logger().info('Finishing rotating.')

            elif (front_lidar_min <= LIDAR_AVOID_DISTANCE): # if the robot has encountered an obstacle in front of it, rotate to the left
                self.cmd.linear.x = 0.0
                self.cmd.angular.z = ROT_VEL
                self.publisher_.publish(self.cmd)
                self.turtlebot_moving = True
                self.stall_count = 0
                self.stall_rotating = True
                self.get_logger().info('Obstacle too close to front of robot. Currently rotating to the left.')

            elif (right_lidar_min > LIDAR_WALL_FAR_DISTANCE): # if the robot is too far away from the right wall
                self.cmd.linear.x = LINEAR_VEL * (10.0 ** linear_speed_calc(-1, right_lidar_min, LIDAR_WALL_FAR_DISTANCE))
                self.cmd.angular.z = ROT_VEL * (0.1 ** linear_speed_calc(-1, right_lidar_min, .7))  # ideally this would be controlled by a PID
                self.get_logger().info('Front distance : %f' % front_lidar_min)
                self.publisher_.publish(self.cmd)
                self.turtlebot_moving = True
                self.stall_count = 0
                self.stall_rotating = True
                self.get_logger().info('FAR: Adjusting closer to the wall. Angular speed is %f' % (0.2 * right_lidar_min))

            elif (right_lidar_min < LIDAR_WALL_SHORT_DISTANCE): # if the robot is too close to the right wall
                self.cmd.linear.x = LINEAR_VEL * (0.1 ** linear_speed_calc(-1, right_lidar_min, LIDAR_WALL_SHORT_DISTANCE))
                self.cmd.angular.z = -ROT_VEL * (10.0 ** linear_speed_calc(-1, right_lidar_min, STOP_DISTANCE)) # ideally this would be controlled by a PID
                self.publisher_.publish(self.cmd)
                self.turtlebot_moving = True
                self.stall_count = 0
                self.stall_rotating = True
                self.get_logger().info('CLOSE: Adjusting farther from the wall. Angular speed is %f' % (0.2 * right_lidar_min))
            
            else:
                self.cmd.linear.x = LINEAR_VEL
                self.cmd.angular.z = 0.0
                self.publisher_.publish(self.cmd)
                self.turtlebot_moving = True
                self.stall_count = 0
                self.stall_rotating = True
                self.get_logger().info('WEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEE')

        self.get_logger().info('Front distance : %f' % front_lidar_min)
        self.get_logger().info('Left distance : %f' % left_lidar_min)
        self.get_logger().info('Right distance : %f' % right_lidar_min)
        # self.get_logger().info('I receive: "%s"' % str(self.odom_data)) 
        #if self.stall == True:
        #   self.get_logger().info('Stall reported')
           # self.cmd.linear.x = 0.0
           # self.cmd.angular.z = 0.5
           #self.turtlebot_moving = True
           # time.sleep(1)

        # Display the message on the console
        # self.get_logger().info('Publishing: "%s"' % self.cmd)
                    #if (self.stall == True): # if the robot is stalling
                    
                    ##if (self.stall_rotating == False):
                    #    self.cmd.angular.z = 0.3                    
                    #    self.publisher_.publish(self.cmd)
                    #    self.cmd.linear.x = -0.8
                    #    self.turtlebot_moving = True
                    #    self.stall_rotating = True
                    #    self.get_logger().info('Reversing out of a stall.')
                    ##else:
                    #    
                    #    self.cmd.linear.x = 0.0
                    #    self.cmd.angular.z = 0.0
                    #    self.publisher_.publish(self.cmd)
                    #    self.turtlebot_moving = True
                    #    self.rotating = True
                    #    self.stall = False
                    #    self.stall_count= 0
                    #    self.get_logger().info('Reversing out of a stall.')
                    #    

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