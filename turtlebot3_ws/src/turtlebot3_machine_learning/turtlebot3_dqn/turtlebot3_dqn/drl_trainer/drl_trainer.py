import math
import numpy
import os

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
# from stable_baselines3 import PPO
from stable_baselines3 import TD3
from stable_baselines3.common.noise import NormalActionNoise, OrnsteinUhlenbeckActionNoise

import time

import gym
from gym import spaces
import sys


# DRLEnv -> gym_NavEnv
# DRLTrainer -> ros_NavEnv
# Trainer ->

class gym_NavEnv(gym.Env):

    def __init__(self, n_actions):  # 5 (2 - a) * 1.5 -> -3 ~ 3
        super(gym_NavEnv, self).__init__()
        # Define action and observation space
        # They must be gym.spaces objects
        # Example when using discrete actions:
        self.n_actions = n_actions
        # self.action_space = spaces.Discrete(n_actions)
        self.action_space = spaces.Box(low=-2, high=2, shape=(2,), dtype="float32")
        self.observation_space = spaces.Box(low=-3.5, high=3.5, shape=(28,), dtype="float32") # 26 + 2(before action)
        print(self.action_space)
        rclpy.init(args=None)
        self.drl_trainer = ros_NavEnv()
        self.goal_count = 0
        self.coll_count = 0
        self.global_count = 0
        self.before_action = [0, 0]
        # rclpy.spin(self.drl_trainer)
        # self.drl_trainer.destroy()
        # rclpy.shutdown()

    def step(self, action):
        for _ in range(10):
            rclpy.spin_once(self.drl_trainer)
        twist = Twist()
        #
        # print(action)
        lv = action[0]  # linear_velocity
        av = action[1]  # angular_velocity
        # print(lv, av)
        # print(lv, av)
        lv += 2  # 0 ~ 4
        lv /= 20 # 0 ~ 0.2
        # twist.linear.x = 0.15
        twist.linear.x = lv
        twist.angular.z = av
        # print(lv, av)
        # twist.angular.z = ((self.n_actions - 1) / 2 - action) * 1.5
        self.drl_trainer.cmd_vel_pub.publish(twist)
        observation = self.drl_trainer.get_state(self.before_action)
        reward = self.drl_trainer.get_reward(action)
        done = self.drl_trainer.done
        if done:
            self.drl_trainer.done = False
            self.drl_trainer.succeed = False
            self.drl_trainer.fail = False
            time.sleep(2)
        info = {}
        self.global_count += 1
        if self.global_count % 5000 * 10 == 0:
            print("current step : " + str(self.global_count) + " goal_count : " + str(
                self.drl_trainer.goal_count) + " coll_count : " + str(
                self.drl_trainer.coll_count) + " goal percent : " + str(
                float(self.drl_trainer.goal_count / (self.drl_trainer.goal_count + self.drl_trainer.coll_count))))
            self.drl_trainer.coll_count = 0
            self.drl_trainer.goal_count = 0
        # step loop rate
        # print(observation)
        # print(action)
        # print(lv, av)
        # print(reward)
        # time.sleep(0.5)
        self.before_action[0] = lv
        self.before_action[1] = av

        return observation, reward, done, info

    def reset(self):
        # time.sleep(15)
        time.sleep(2)
        if self.drl_trainer.done:
            self.drl_trainer.done = False
            self.drl_trainer.succeed = False
            self.drl_trainer.fail = False
        for _ in range(10):
            rclpy.spin_once(self.drl_trainer)
        self.drl_trainer.init_goal_distance = math.sqrt(
            (self.drl_trainer.goal_pose_x - self.drl_trainer.last_pose_x) ** 2
            + (self.drl_trainer.goal_pose_y - self.drl_trainer.last_pose_y) ** 2)
        observation = self.drl_trainer.reset(self.before_action)

        return observation  # reward, done, info can't be included

    def render(self, mode='human'):
        pass

    def close(self):
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

        self.goal_count = 0
        self.coll_count = 0

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
        # self.map_sub = self.create_subscription(
        #     OccupancyGrid,  # msg_type same
        #     'map',  # map_info willbe
        #     self.map_callback,
        #     qos)
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
    # def map_callback(self, msg):
    #     print(msg)
    #     # 7568 = 86 * 88
    #     print(len(msg.data))

    ######

    def goal_pose_callback(self, msg):
        self.goal_pose_x = msg.position.x
        self.goal_pose_y = msg.position.y

    def odom_callback(self, msg):
        self.last_pose_x = msg.pose.pose.position.x
        self.last_pose_y = msg.pose.pose.position.y
        _, _, self.last_pose_theta = self.euler_from_quaternion(msg.pose.pose.orientation)

        goal_distance = math.sqrt(
            (self.goal_pose_x - self.last_pose_x) ** 2
            + (self.goal_pose_y - self.last_pose_y) ** 2)

        path_theta = math.atan2(
            self.goal_pose_y - self.last_pose_y,
            self.goal_pose_x - self.last_pose_x)

        goal_angle = path_theta - self.last_pose_theta
        if goal_angle > math.pi:
            goal_angle -= 2 * math.pi

        elif goal_angle < -math.pi:
            goal_angle += 2 * math.pi

        self.goal_distance = goal_distance
        self.goal_angle = goal_angle
        # print(self.goal_distance)

    def scan_callback(self, msg):
        self.scan_ranges = msg.ranges
        self.min_obstacle_distance = min(self.scan_ranges)
        self.min_obstacle_angle = numpy.argmin(self.scan_ranges)

    def get_state(self, extraargs):
        # state scaling
        pre_state = []
        for i in range(24):
            # print(int((len(self.scan_ranges)/24)*i))
            pre_state.append(self.scan_ranges[int((len(self.scan_ranges) / 24) * i)])
        # pre_state = self.scan_ranges[0::15]
        state = []
        # print(self.goal_distance, self.goal_angle)

        # print(state)
        state.append(float(self.goal_distance))
        state.append(float(self.goal_angle))
        for scan in pre_state:
            if scan == numpy.inf:
                scan = 3.5
            # scan = scan / 3.5
            # if scan > 1:
            #     scan = 1
            # elif scan < -1:
            #     scan = -1
            if scan < 3.5:
                pass
            else:
                scan = 3.5
            state.append(scan)
        # print(state)
        # print(state)
        # for data in state:
        #     if not (data >= -1 and data <= 1):
        #         print(state)
        state.append(extraargs[0])
        state.append(extraargs[1])
        #print(state)
        self.local_step += 1

        # Succeed
        if self.goal_distance < 0.15:  # unit: m
            print("Goal! :)")
            self.succeed = True
            self.done = True
            self.cmd_vel_pub.publish(Twist())  # robot stop
            self.local_step = 0
            self.goal_count += 1
            req = Empty.Request()
            while not self.task_succeed_client.wait_for_service(timeout_sec=1.0):
                self.get_logger().info('service not available, waiting again...')
            self.task_succeed_client.call_async(req)

        # Fail
        if self.min_obstacle_distance < 0.15:  # unit: m
            print("Collision! :(")
            self.fail = True
            self.done = True
            self.cmd_vel_pub.publish(Twist())  # robot stop
            self.local_step = 0
            self.coll_count += 1
            req = Empty.Request()
            while not self.task_fail_client.wait_for_service(timeout_sec=1.0):
                self.get_logger().info('service not available, waiting again...')
            self.task_fail_client.call_async(req)

            # os.system('killall occupancy_grid_')
            # os.system('killall cartographer_no')
            # os.system('killall rviz2')
            # time.sleep(5) # rviz2

        if self.local_step == 500:
            print("Time out! :(")
            self.fail = True
            self.done = True
            self.cmd_vel_pub.publish(Twist())  # robot stop
            self.local_step = 0
            self.coll_count += 1
            req = Empty.Request()
            while not self.task_fail_client.wait_for_service(timeout_sec=1.0):
                self.get_logger().info('service not available, waiting again...')
            self.task_fail_client.call_async(req)

        return state

    def reset(self, extraargs):
        return self.get_state(extraargs)

    def get_reward(self, action):
        # yaw_reward = 1 - 2 * math.sqrt(math.fabs(self.goal_angle / math.pi))
        #
        # distance_reward = (2 * self.init_goal_distance) / \
        #                   (self.init_goal_distance + self.goal_distance) - 1
        #
        # # Reward for avoiding obstacles
        # if self.min_obstacle_distance < 0.25:
        #     obstacle_reward = -2
        # else:
        #     obstacle_reward = 0
        #
        # reward = yaw_reward + distance_reward + obstacle_reward
        #
        # # + for succeed, - for fail
        # if self.succeed:
        #     # print(self.succeed)
        #     reward += 5
        #     # print(reward)
        # elif self.fail:
        #     # print(self.fail)
        #     reward += -10
        #     # print(reward)
        # # print(self.succeed)
        # # print(self.fail)
        # # print(yaw_reward)
        # # print(distance_reward)
        # # print(obstacle_reward)
        # #print(reward)
        yaw_reward = []
        obstacle_min_range = self.min_obstacle_distance
        current_distance = self.goal_distance
        # heading = state[-4]

        # for i in range(5):
        #    angle = -math.pi / 4 + self.goal_angle + (math.pi / 8 * i) + math.pi / 2
        #    tr = 1 - 4 * math.fabs(0.5 - math.modf(0.25 + 0.5 * angle % (2 * math.pi) / math.pi)[0])
        #    yaw_reward.append(tr)

        distance_rate = -2 ** (current_distance / self.init_goal_distance)

        if obstacle_min_range < 0.2:
            ob_reward = -5
        else:
            ob_reward = 0

        # action[1] angular velocity, -3 ~ 3 -> 0 1 2 3 4
        i = action[0]
        i += 3
        i = (i * 2) / 3

        angle = -math.pi / 4 + self.goal_angle + (math.pi / 8 * i) + math.pi / 2
        tr = 1 - 4 * math.fabs(0.5 - math.modf(0.25 + 0.5 * angle % (2 * math.pi) / math.pi)[0])
        reward = 0
        yaw_reward = tr
        # reward = ((round(yaw_reward[action] * 5, 2)) * distance_rate) + ob_reward
        # print(yaw_reward, distance_rate)
        # reward = round(yaw_reward*5, 2) * 0.1 + distance_rate * 10
        # reward = distance_rate
        # reward -= 0.1
        lv = action[0]  # linear_velocity
        av = action[1]  # angular_velocity
        # print(lv, av)
        # print(lv, av)
        lv += 2  # 0 ~ 4
        lv /= 20 # 0 ~ 0.2
        # dist_reward = 1 - current_distance**0.4
        # vel_discount = (1 - max(lv, 0.1))**(1/max(current_distance, 0.1))
        # reward = vel_discount * dist_reward
        if self.succeed:
            # print(self.succeed)
            reward = 200
            # print(reward)
        elif self.fail:
            # print(self.fail)
            reward = -150
        # elif obstacle_min_range < 0.15 + 0.15: # radius robot + safety
        #     reward = 1 - (current_distance / (0.15 + 0.15))
        # else: # scale factor 1
        #     reward = 1 * (self.init_goal_distance - current_distance)
        # else:
        #     r3 = lambda x: 1 - x if x < 1 else 0.0
        #     reward = action[0] / 2 - abs(action[1]) / 2 - r3(obstacle_min_range) / 2
        else:
           reward = 1 - numpy.exp(current_distance*0.99) # discount factor 0.99
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

        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = numpy.arctan2(sinr_cosp, cosr_cosp)

        sinp = 2 * (w * y - z * x)
        pitch = numpy.arcsin(sinp)

        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = numpy.arctan2(siny_cosp, cosy_cosp)

        return roll, pitch, yaw

    def dqn_com_callback(self, request, response):
        action = request.action
        twist = Twist()
        twist.linear.x = 0.15
        twist.angular.z = ((self.action_size - 1) / 2 - action) * 1.5
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
                (self.goal_pose_x - self.last_pose_x) ** 2
                + (self.goal_pose_y - self.last_pose_y) ** 2)

        return response


class Trainer():
    def __init__(self, mode):
        # super().__init__('trainer')
        print("0")
        self.env = gym_NavEnv(n_actions=5)
        print("1")
        if mode == "i":
            self.inferencing()
        else:
            self.training()
        print("2")

    def training(self):
        # The noise objects for TD3
        n_actions = self.env.action_space.shape[-1]
        print(n_actions)
        action_noise = NormalActionNoise(mean=numpy.zeros(n_actions), sigma=0.1 * numpy.ones(n_actions))
        # model = PPO("MlpPolicy", self.env, verbose=1)
        # model = TD3("MlpPolicy", self.env, action_noise=action_noise, verbose=1)
        # model.learn(total_timesteps=100000, log_interval=5000)
        # model.save("td3_")

        model = TD3("MlpPolicy", self.env, action_noise=action_noise, verbose=1)
        # model = PPO.load(path="result_ppo/ppo_r1_2200000", env=self.env)
        # model = TD3.load(path="result_td3/td3_100000", env=self.env, action_noise=action_noise, verbose=1)
        model.learn(total_timesteps=100000, log_interval=5000)
        model.save("result_td3/td3_r3_100000")
        result_folder = "result_td3/"

        for i in range(10):
            logname = "td3_r3_" + str((i + 2) * 100000 + 0)
            model.learn(total_timesteps=100000,
                        reset_num_timesteps=False,
                        tb_log_name=logname)

            # self.env.close()

            path = result_folder + logname
            model.save(path)

    def inferencing(self):
        # model = PPO("MlpPolicy", self.env, verbose=1)
        n_actions = self.env.action_space.shape[-1]
        print(n_actions)
        action_noise = NormalActionNoise(mean=numpy.zeros(n_actions), sigma=0.1 * numpy.ones(n_actions))
        model = TD3.load(path="result_td3/td3_100000", env=self.env, action_noise=action_noise, verbose=1)

        obs = self.env.reset()
        while True:
            action, _states = model.predict(obs)
            obs, rewards, dones, info = self.env.step(action)
            if dones:
                time.sleep(10)


def main(args=sys.argv[1]):
    Trainer(args)


if __name__ == '__main__':
    main()
