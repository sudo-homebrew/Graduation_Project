import math
import numpy

from geometry_msgs.msg import Pose
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_srvs.srv import Empty
from nav_msgs.msg import OccupancyGrid
import sys
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSReliabilityPolicy
from turtlebot3_msgs.srv import Dqn
import time
from rclpy.qos import ReliabilityPolicy, QoSProfile


class Zigzag(Node):
    def __init__(self, stage):
        super().__init__('zigzag')

        """************************************************************
        ** Initialise variables
        ************************************************************"""
        self.goal_pose_x = 0.0
        self.goal_pose_y = 0.0
        self.last_pose_x = 0.0
        self.last_pose_y = 0.0
        self.last_pose_theta = 0.0

        self.action_size = 5
        self.done = False
        self.fail = False
        self.succeed = False
        self.time_over = False

        self.before_dist = 1.0
        self.goal_angle = 0.0
        self.goal_distance = 1.0
        self.init_goal_distance = 1.0
        self.scan_ranges = []
        self.min_obstacle_distance = 10.0
        self.forward_distance = 1.0
        self.min_obstacle_angle = 10.0

        self.local_step = 0

        """************************************************************
        ** Initialise ROS publishers and subscribers
        ************************************************************"""
        qos = QoSProfile(depth=10)
        qos_profile = QoSProfile(depth=10)
        qos_profile.reliability = QoSReliabilityPolicy.BEST_EFFORT
        qos_profile.durability = QoSDurabilityPolicy.VOLATILE

        # Initialise publishers
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', qos)

        # Initialise subscribers
        self.goal_pose_sub = self.create_subscription(
            Pose,
            'goal_pose',
            self.goal_pose_callback,
            qos)
        self.odom_sub = self.create_subscription(
            Odometry,
            'odom',
            self.odom_callback,
            qos)
        self.scan_sub = self.create_subscription(
            LaserScan,
            'scan',
            self.scan_callback,
            qos_profile=qos_profile_sensor_data)
        
        #####################
        self.map_sub = self.create_subscription(
            OccupancyGrid,            
            'map',
            self.map_callback,
            qos)
      	####################
	
        # Initialise client
        self.task_succeed_client = self.create_client(Empty, 'task_succeed')
        self.task_fail_client = self.create_client(Empty, 'task_fail')

        # Initialise servers
        self.dqn_com_server = self.create_service(Dqn, 'dqn_com', self.dqn_com_callback)
        
        
        #self.process()
        #self.get_

    """*******************************************************************************
    ** Callback functions and relevant functions
    *******************************************************************************"""
    def goal_pose_callback(self, msg):
        self.goal_pose_x = msg.position.x
        self.goal_pose_y = msg.position.y

    def odom_callback(self, msg):
        self.last_pose_x = msg.pose.pose.position.x
        self.last_pose_y = msg.pose.pose.position.y
        _, _, self.last_pose_theta = self.euler_from_quaternion(msg.pose.pose.orientation)

        goal_distance = math.sqrt(
            (self.goal_pose_x-self.last_pose_x)**2
            + (self.goal_pose_y-self.last_pose_y)**2)

        path_theta = math.atan2(
            self.goal_pose_y-self.last_pose_y,
            self.goal_pose_x-self.last_pose_x)

        goal_angle = path_theta - self.last_pose_theta
        if goal_angle > math.pi:
            goal_angle -= 2 * math.pi

        elif goal_angle < -math.pi:
            goal_angle += 2 * math.pi
        
        self.before_dist = self.goal_distance
        self.goal_distance = goal_distance
        self.goal_angle = goal_angle
        #time.sleep(0.1)

    def scan_callback(self, msg):
        self.scan_ranges = msg.ranges
        #temp = self.scan_ranges[0::45]
        #print(len(temp))
        self.forward_distance = self.scan_ranges[0]
        self.min_obstacle_distance = min(self.scan_ranges)
        self.min_obstacle_angle = numpy.argmin(self.scan_ranges)
        #print(self.forward_distance)
        #time.sleep(0.1)
        #print(self.min_obstacle_distance)
        
    ######
    def map_callback(self, msg):
        print(msg)
        # 7568 = 86 * 88
        print(len(msg.data))
    ######

    def get_state(self):
        state = self.scan_ranges[0::15]
        for i in range(len(state)):
            if state[i] == float('Inf'):
                state[i] = 3.5
            elif numpy.isnan(state[i]):
                state[i] = 0
        state.append(float(self.goal_distance))
        state.append(float(self.goal_angle))
        #print(self.goal_distance)
        print(self.goal_angle)
   
        self.local_step += 1
        #print(state)

        # Succeed
        if self.goal_distance < 0.20:  # unit: m
            print("Goal! :)")
            self.succeed = True
            self.done = True
            self.cmd_vel_pub.publish(Twist())  # robot stop
            #self.local_step = 0
            req = Empty.Request()
            while not self.task_succeed_client.wait_for_service(timeout_sec=1.0):
                self.get_logger().info('service not available, waiting again...')
            self.task_succeed_client.call_async(req)
            time.sleep(0.1)

        # Fail
        if self.min_obstacle_distance < 0.13:  # unit: m
            print("Collision! :(")
            self.fail = True
            self.done = True
            self.cmd_vel_pub.publish(Twist())  # robot stop
            self.local_step = 0
            req = Empty.Request()
            while not self.task_fail_client.wait_for_service(timeout_sec=1.0):
                self.get_logger().info('service not available, waiting again...')
            self.task_fail_client.call_async(req)
            time.sleep(0.1)

        if self.local_step == 500:
            print("Time out! :(")
            #self.fail = True
            self.time_over = True
            self.done = True
            self.local_step = 0
            self.cmd_vel_pub.publish(Twist())  # robot stop
            #req = Empty.Request()
            #while not self.task_fail_client.wait_for_service(timeout_sec=1.0):
            #    self.get_logger().info('service not available, waiting again...')
            #self.task_fail_client.call_async(req)
        return state

    def dqn_com_callback(self, request, response):
        action = request.action
        twist = Twist()
        twist.linear.x = 0.15
        twist.angular.z = ((self.action_size - 1)/2 - action) * 1.5
        self.cmd_vel_pub.publish(twist)
        time.sleep(0.1)
        self.cmd_vel_pub.publish(Twist())

        response.state = self.get_state()
        response.reward = self.get_reward(action)
        response.done = self.done
        
        #response.done = self.done
        #response.done = 0 if self.done else 1

        if self.succeed:
            self.succeed = False
            print("inittrue")
            self.init_goal_distance = math.sqrt(
                (self.goal_pose_x-self.last_pose_x)**2
                + (self.goal_pose_y-self.last_pose_y)**2)

        if self.done is True:
            self.done = False
            self.succeed = False
            self.fail = False
            self.time_over = False
        
        if request.init is True:
            print("inittrue")
            self.init_goal_distance = math.sqrt(
                (self.goal_pose_x-self.last_pose_x)**2
                + (self.goal_pose_y-self.last_pose_y)**2)

        return response

    def get_reward(self, action):
        yaw_reward = []
        #current_distance = self.goal_distance
        #heading = self.goal_angle

        for i in range(5):
            angle = -math.pi / 4 + self.goal_angle + (math.pi / 8 * i) + math.pi / 2
            tr = 1 - 4 * math.fabs(0.5 - math.modf(0.25 + 0.5 * angle % (2 * math.pi) / math.pi)[0])
            yaw_reward.append(tr)

        distance_rate = 2 ** (self.init_goal_distance / (self.goal_distance+4))
        reward = ((round(yaw_reward[action] * 5, 2)) * distance_rate) * 0.1
        

        #yaw_reward = 1 - 2*math.sqrt(math.fabs(self.goal_angle / math.pi))

        #distance_reward = (2 * self.init_goal_distance) / \
        #    (self.init_goal_distance + self.goal_distance) - 1

        # Reward for avoiding obstacles
        #obstacle_reward = -0.1
        #reward = reward - 0.1
        #if self.min_obstacle_distance < 0.5:
        #    obstacle_reward = -3.0
        #else:
        #    obstacle_reward = -0.1
        #print(distance_reward)
        #print(yaw_reward)
        #reward = yaw_reward + distance_reward + obstacle_reward
        #reward = self.init_goal_distance - self.goal_distance
        reward = -0.1
        dist = self.goal_distance - self.before_dist
        reward = -dist * 50 - self.goal_distance / 5 - math.fabs(self.goal_angle)
        print("reward : " + str(reward))
        print(-dist * 50)
        print(-self.goal_distance / 5)
        print(-math.fabs(self.goal_angle))

        #if reward > 10 or reward < -10:
        #    print("Abnormal Detect")
        #    print("yaw_reward:" + str((round(yaw_reward[action] * 5, 2))))
        #    print("distance_rate:" + str(distance_rate))

        # + for succeed, - for fail
        if self.succeed:
            reward += 100
            self.succeed = False
        if self.fail:
            reward = reward - 100
            self.fail = False
        if self.time_over:
            reward = reward - 100
            self.time_over = False
        #print(reward)
        #if reward != -0.1:
            #print(dist)
            #print(reward)
       
        return reward

    """*******************************************************************************
    ** Below should be replaced when porting for ROS 2 Python tf_conversions is done.
    *******************************************************************************"""
    def euler_from_quaternion(self, quat):
        """
        Converts quaternion (w in last place) to euler roll, pitch, yaw
        quat = [x, y, z, w]
        """
        x = quat.x
        y = quat.y
        z = quat.z
        w = quat.w

        sinr_cosp = 2 * (w*x + y*z)
        cosr_cosp = 1 - 2*(x*x + y*y)
        roll = numpy.arctan2(sinr_cosp, cosr_cosp)

        sinp = 2 * (w*y - z*x)
        pitch = numpy.arcsin(sinp)

        siny_cosp = 2 * (w*z + x*y)
        cosy_cosp = 1 - 2 * (y*y + z*z)
        yaw = numpy.arctan2(siny_cosp, cosy_cosp)

        return roll, pitch, yaw

    def process(self): # init left down (-2.0, 2.0) upward direction, while steping step size % 4
        steping = True
        count = 0
        while steping:
            print(count)
            self.forwarding()
            self.cmd_vel_pub.publish(Twist())
            print("forwarding over")
            time.sleep(1)
            steping = self.turning(count)                                      
            print("turning over")
            count = count + 1
            
                
    def forwarding(self):
        twist = Twist()
        twist.linear.x = 0.15
        twist.angular.z = ((5 - 1)/2 - 2) * 1.5
        self.cmd_vel_pub.publish(twist)
        while rclpy.ok():
            rclpy.spin_once(self)
            time.sleep(0.1)
            if not self.checking_wall():
                break
        return 0

    def turning(self, count): # get LSD(only for forward distance in this case)
                 
        #if count % 4 == 0:
        #    self.robot_moving(vel=0.0, action=4, times=1) # right90            
        #    self.robot_moving(vel=0.15, action=2, times=3) # forwarding 0.45(approximate robot size)
        #    if not self.checking_wall(): #finish
        #        self.cmd_vel_pub.publish(Twist())            
        #elif count % 4 == 1:
        #    self.robot_moving(vel=0.0, action=4, times=1) # right90
            
        #elif count % 4 == 2:
        #    self.robot_moving(vel=0.0, action=0, times=1) # left90
        #    self.robot_moving(vel=0.15, action=2, times=3) # forwarding 0.45(approximate robot size)
        #    if not self.checking_wall():
        #        self.cmd_vel_pub.publish(Twist())
            
        #elif count % 4 == 3:
        #    self.robot_moving(vel=0.0, action=0, times=1) # left90
            
        #else:
        #    print("error")
        #rclpy.spin_once(self)
        #time.sleep(0.1)
        if count % 2 == 0:
            
            self.robot_moving(vel=0.0, action=4, times=1) # right90
            for i in range(15):
                rclpy.spin_once(self)
                time.sleep(0.1)
            if not self.checking_wall(): #finish
                self.cmd_vel_pub.publish(Twist())
                return False           
            self.robot_moving(vel=0.15, action=2, times=3) # forwarding 0.45(approximate robot size)
            time.sleep(1)
            self.robot_moving(vel=0.0, action=4, times=1) # right90
            for i in range(15):
                rclpy.spin_once(self)
                time.sleep(0.1)
            if not self.checking_wall(): #finish
                self.cmd_vel_pub.publish(Twist())
                return False       
        elif count % 2 == 1:
            self.robot_moving(vel=0.0, action=0, times=1) # left90
            for i in range(15):
                rclpy.spin_once(self)
                time.sleep(0.1)
            if not self.checking_wall():
                self.cmd_vel_pub.publish(Twist())
                return False
            self.robot_moving(vel=0.15, action=2, times=3) # forwarding 0.45(approximate robot size)
            time.sleep(1)            
            self.robot_moving(vel=0.0, action=0, times=1) # left90
            for i in range(15):
                rclpy.spin_once(self)
                time.sleep(0.1)
            if not self.checking_wall():
                self.cmd_vel_pub.publish(Twist())
                return False
        else:
            print("error")
        return True
        

    def checking_wall(self):             
        print(self.forward_distance)
        if self.forward_distance < 0.5:  # unit: m
            print("dist err")      
            return False
        return True

    def robot_moving(self, vel=0.15, action=2, times=1): # action 0, 1, 2, 3, 4 left90, left45, forward, right45, right90
        twist = Twist()
        twist.linear.x = vel
        twist.angular.z = ((self.action_size - 1)/2 - action) * 90 * math.pi / 360 # 2 -2
        #print(twist.angular.z)
        self.cmd_vel_pub.publish(twist)
        time.sleep(times)
        self.cmd_vel_pub.publish(Twist())

def main(args=sys.argv[1]):
    rclpy.init(args=args)
    zigzag = Zigzag(args)
    rclpy.spin(zigzag)

    zigzag.destroy()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
