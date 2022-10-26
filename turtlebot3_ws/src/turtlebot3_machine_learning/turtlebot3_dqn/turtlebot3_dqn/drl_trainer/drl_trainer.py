import math
import numpy

from geometry_msgs.msg import Pose
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from nav_msgs.msg import OccupancyGrid
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_srvs.srv import Empty

from turtlebot3_msgs.srv import Dqn
from stable_baselines3 import PPO

import time


import gym
from gym import spaces


# import matlabs engine

import matlab.engine


# DRLEnv -> gym_NavEnv
# DRLTrainer -> ros_NavEnv
# Trainer ->

class gym_NavEnv(gym.Env):

    def __init__(self, n_actions):
        super(gym_NavEnv, self).__init__()
        # Define action and observation space
        # They must be gym.spaces objects
        # Example when using discrete actions:
        self.n_actions = n_actions
        self.action_space = spaces.Discrete(n_actions)
        self.observation_space = spaces.Box(low=-1, high=1, shape=(10,), dtype=numpy.uint8)
        print("Launching MATLAB")
        self.matlab_slam = matlab.engine.start_matlab()
        print("MATLAB has launched")
        # How to use PySLAM
        # PySLAM(ROS_DOMAIN_ID, ROS2DomainID, Duration)
        result = self.matlab_slam.PySLAM(30, 'matlab_example_robot', 10)
        
        print("Starting PySLAM")
        rclpy.init(args=None)
        self.drl_trainer = ros_NavEnv()
        #rclpy.spin(self.drl_trainer)
        #self.drl_trainer.destroy()
        #rclpy.shutdown()
    def step(self, action):
        for _ in range(10):
            rclpy.spin_once(self.drl_trainer)
        twist = Twist()
        twist.linear.x = 0.3
        twist.angular.z = ((self.n_actions - 1) / 2 - action) * 1.5
        self.drl_trainer.cmd_vel_pub.publish(twist)
        observation = self.drl_trainer.get_state()
        reward = self.drl_trainer.get_reward(action)
        done = self.drl_trainer.done
        if self.drl_trainer.done:
            self.drl_trainer.done = False
            self.drl_trainer.succeed = False
            self.drl_trainer.fail = False
        info = {}
        if done:
            time.sleep(1)
        occmap = self.matlab_slam.PyGetMap()
        print(occmap)
        # step loop rate
        time.sleep(0.1)

        return observation, reward, done, info
    def reset(self):
        time.sleep(1)
        if self.drl_trainer.done:
            self.drl_trainer.done = False
            self.drl_trainer.succeed = False
            self.drl_trainer.fail = False
        for _ in range(10):
            rclpy.spin_once(self.drl_trainer)
        self.drl_trainer.init_goal_distance = math.sqrt(
            (self.drl_trainer.goal_pose_x - self.drl_trainer.last_pose_x) ** 2
            + (self.drl_trainer.goal_pose_y - self.drl_trainer.last_pose_y) ** 2)
        observation = self.drl_trainer.reset()
        self.matlab_slam.PyResetMap()
        occmap = self.matlab_slam.PyGetMap()
        print(occmap)
        return observation  # reward, done, info can't be included
    def render(self, mode='human'):
        pass
    def close (self):
        self.drl_trainer.destroy()
        rclpy.shutdown()
        pass

class ros_NavEnv(Node):
    def __init__(self):
        super().__init__('drl_trainer')
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

        self.goal_angle = 0.0
        self.goal_distance = 1.0
        self.init_goal_distance = 1.0
        self.scan_ranges = []
        self.min_obstacle_distance = 10.0
        self.min_obstacle_angle = 10.0

        self.local_step = 0

        """************************************************************
        ** Initialise ROS publishers and subscribers
        ************************************************************"""
        qos = QoSProfile(depth=10)

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

        # Matlab SLAM
        #####################
        self.map_sub = self.create_subscription(
            OccupancyGrid, #msg_type same
            'map', # map_info willbe
            self.map_callback,
            qos)
        ####################

        # Initialise client
        self.task_succeed_client = self.create_client(Empty, 'task_succeed')
        self.task_fail_client = self.create_client(Empty, 'task_fail')

        # Initialise servers
        self.dqn_com_server = self.create_service(Dqn, 'dqn_com', self.dqn_com_callback)

    """*******************************************************************************
    ** Callback functions and relevant functions
    *******************************************************************************"""
    ######
    def map_callback(self, msg):
        print(msg)
        # 7568 = 86 * 88
        print(len(msg.data))
    ######

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

        self.goal_distance = goal_distance
        self.goal_angle = goal_angle

    def scan_callback(self, msg):
        self.scan_ranges = msg.ranges
        self.min_obstacle_distance = min(self.scan_ranges)
        self.min_obstacle_angle = numpy.argmin(self.scan_ranges)

    def get_state(self):
        # state scaling
        pre_state = self.scan_ranges[0::45]
        state = []

        #print(state)
        state.append(float(self.goal_distance / 3.5))
        state.append(float(self.goal_angle / 3.5))
        for scan in pre_state:
            if scan == numpy.inf:
                scan = 3.5
            scan = scan / 3.5
            if scan > 1:
                scan = 1
            elif scan < -1:
                scan = -1
            state.append(scan)
        #print(state)
        #print(state)
        # for data in state:
        #     if not (data >= -1 and data <= 1):
        #         print(state)
        self.local_step += 1

        # Succeed
        if self.goal_distance < 0.20:  # unit: m
            print("Goal! :)")
            self.succeed = True
            self.done = True
            self.cmd_vel_pub.publish(Twist())  # robot stop
            self.local_step = 0
            req = Empty.Request()
            while not self.task_succeed_client.wait_for_service(timeout_sec=1.0):
                self.get_logger().info('service not available, waiting again...')
            self.task_succeed_client.call_async(req)

        # Fail
        if self.min_obstacle_distance < 0.20:  # unit: m
            print("Collision! :(")
            self.fail = True
            self.done = True
            self.cmd_vel_pub.publish(Twist())  # robot stop
            self.local_step = 0
            req = Empty.Request()
            while not self.task_fail_client.wait_for_service(timeout_sec=1.0):
                self.get_logger().info('service not available, waiting again...')
            self.task_fail_client.call_async(req)

        if self.local_step == 5000:
            print("Time out! :(")
            self.fail = True
            self.done = True
            self.cmd_vel_pub.publish(Twist())  # robot stop
            self.local_step = 0
            req = Empty.Request()
            while not self.task_fail_client.wait_for_service(timeout_sec=1.0):
                self.get_logger().info('service not available, waiting again...')
            self.task_fail_client.call_async(req)

        return state

    def reset(self):
        return self.get_state()

    def dqn_com_callback(self, request, response):
        action = request.action
        twist = Twist()
        twist.linear.x = 0.3
        twist.angular.z = ((self.action_size - 1)/2 - action) * 1.5
        self.cmd_vel_pub.publish(twist)

        response.state = self.get_state()
        response.reward = self.get_reward(action)
        response.done = self.done

        if self.done is True:
            self.done = False
            self.succeed = False
            self.fail = False

        if request.init is True:
            self.init_goal_distance = math.sqrt(
                (self.goal_pose_x-self.last_pose_x)**2
                + (self.goal_pose_y-self.last_pose_y)**2)

        return response

    def get_reward(self, action):
        yaw_reward = 1 - 2*math.sqrt(math.fabs(self.goal_angle / math.pi))

        distance_reward = (2 * self.init_goal_distance) / \
            (self.init_goal_distance + self.goal_distance) - 1

        # Reward for avoiding obstacles
        if self.min_obstacle_distance < 0.25:
            obstacle_reward = -2
        else:
            obstacle_reward = 0

        reward = yaw_reward + distance_reward + obstacle_reward

        # + for succeed, - for fail
        if self.succeed:
            #print(self.succeed)
            reward += 5
            #print(reward)
        elif self.fail:
            #print(self.fail)
            reward += -10
            #print(reward)
        #print(self.succeed)
        #print(self.fail)
        #print(yaw_reward)
        #print(distance_reward)
        #print(obstacle_reward)
        print(reward)
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

class Trainer():
    def __init__(self):
        #super().__init__('trainer')
        print("0")
        self.env = gym_NavEnv(n_actions=5)
        print("1")
        self.training()
        print("2")

    def training(self):
        model = PPO("MlpPolicy", self.env, verbose=1)
        model.learn(total_timesteps=5000)
        model.save("ppo_")

def main(args=None):
    Trainer()

if __name__ == '__main__':
    main()
