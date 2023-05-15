#!/usr/bin/env python3
#


import math
import time
import signal
import numpy as np
import matplotlib.pyplot as plt
import sys
import os
import functools
import logging
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import tf as tranf
#from tf.transformations import euler_from_quaternion, quaternion_from_euler
from pyquaternion import Quaternion
from collections import deque
from .real_goal import RealGoal

def readpgm(name):
    with open(name) as f:
        lines = f.readlines()
    # This ignores commented lines
    for l in list(lines):
        if l[0] == '#':
            lines.remove(l)
    # here,it makes sure it is ASCII format (P2)
    assert lines[0].strip() == 'P2'
    # Converts data to a list of integers
    data = []
    for line in lines[1:]:
        data.extend([int(c) for c in line.split()])
    return (np.array(data[3:]),(data[1],data[0]),data[2])


class SensorSub(Node):
    def __init__(self, pose):
        super().__init__('SensorSub')

        """************************************************************
        ** Initialise variables
        ************************************************************"""
        self.scan_ranges = []
        self.scanMsg = 0
        
        self.goal_pose_x = pose[0]
        self.goal_pose_y = pose[1]
        self.last_pose_x = 0.0
        self.last_pose_y = 0.0
        self.last_pose_theta = 0.0
        
        """************************************************************
        ** Initialise ROS publishers and subscribers
        ************************************************************"""
        qos = QoSProfile(depth=10)

        # Initialise subscribers
        self.scan_sub = self.create_subscription(
            LaserScan,
            'scan',
            self.scan_callback,
            qos_profile=qos_profile_sensor_data)
            
        self.odom_sub = self.create_subscription(
            Odometry,
            'odom',
            self.odom_callback,
            qos)

    """*******************************************************************************
    ** Callback functions and relevant functions
    *******************************************************************************"""

    def scan_callback(self, msg):
        self.scan_ranges = msg.ranges # 0 ~ 359 list if 3.5 > = inf
        self.scanMsg = msg
        
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
        roll = np.arctan2(sinr_cosp, cosr_cosp)

        sinp = 2 * (w*y - z*x)
        pitch = np.arcsin(sinp)

        siny_cosp = 2 * (w*z + x*y)
        cosy_cosp = 1 - 2 * (y*y + z*z)
        yaw = np.arctan2(siny_cosp, cosy_cosp)

        return roll, pitch, yaw
        
    def quaternion_from_euler(roll, pitch, yaw):
        """
        Converts Euler angles to quaternion (w, x, y, z) representation.

        Args:
            roll (float): Roll angle in radians.
            pitch (float): Pitch angle in radians.
            yaw (float): Yaw angle in radians.

        Returns:
            Tuple[float, float, float, float]: Quaternion representation of the input Euler angles.
        """
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)

        w = cy * cp * cr + sy * sp * sr
        x = cy * cp * sr - sy * sp * cr
        y = sy * cp * sr + cy * sp * cr
        z = sy * cp * cr - cy * sp * sr

        return w, x, y, z

def calcqxqy(dist, angl, ang):
    angl = math.radians(angl)
    angle = angl + ang
    if angle > np.pi:
        angle = np.pi - angle
        angle = -np.pi - angle
    if angle < -np.pi:
        angle = -np.pi - angle
        angle = np.pi - angle
    if angle > 0:
        qx, qy = rotatepos([dist, 0], angle)
    else:
        qx, qy = rotateneg([dist, 0], -angle)
    return qx, qy
    
###############################################################
#eul = euler_from_quaternion(rot)
#q9c = Quaternion(axis=[0.0, 0.0, 1.0], radians=eul[2])
###############################################################

class ImplementEnv:
    def __init__(self, global_goal):
        self.node = deque(maxlen = 100)
        self.global_goal_x = global_goal[0]
        self.global_goal_y = global_goal[1]
        self.step_time = 10 # second
        self.threshold = 1 # for detecting gap
        self.is_in_process = True
        
    def step(self, lidar, pos_x, pos_y):
        # Check if robot is reached to Gloabl goal enough
        if self.global_euclidean_distance(pos_x, pos_y) < 1:
        # if distance between global goal and robots location is less than 1m
            self.is_in_process = False
            return [self.global_goal_x, self.global_goal_y]
        
        # condition 1. value difference between 2 sequantial lidar sensor values
        pre_value = lidar[0]
        is_gap = False
        gap_start = 0
        gap_end = 0
        gap_mid = 0
        
        for index, cur_value in enumerate(lidar):
            if abs(pre_value - cur_value) > self.threshold:
                if not is_gap:
                    gap_start = index
                    is_gap = True
                else:
                    gap_end = index
                    is_gap = False
                    
                    gap_mid = int((gap_start + gap_end) / 2)
                    candidate_x, candidate_y = self.calc_lidar_coordinate(gap_mid, lidar[gap_mid], pos_x, pos_y)
                    self.node.append([candidate_x, candidate_y])
                    
                    gap_start = 0
                    gap_end = 0
                    gap_mid = 0
        # condition 2. infinite readings of lidar sensor values
        pre_value = lidar[0]
        is_gap = False
        gap_start = 0
        gap_end = 0
        gap_mid = 0
        
        for index, cur_value in enumerate(lidar):
            if cur_value == math.inf and not is_gap:
                gap_start = index
                is_gap = True
                
            if cur_value != math.inf and is_gap:
                gap_end = index
                is_gap = False
                
                gap_mid = int((gap_start + gap_end) / 2)
                candidate_x, candidate_y = self.calc_lidar_coordinate(gap_mid, lidar[gap_mid], pos_x, pos_y)
                self.node.append([candidate_x, candidate_y])
                
                gap_start = 0
                gap_end = 0
                gap_mid = 0
        
        ######################
        # TODO
        # Match lidar with map
        ######################
        
        heuristic_scores = self.heuristic(pos_x, pos_y)
        
        try:
            POI = self.node[heuristic_scores.index(min(heuristic_scores))]
        except ValueError:
            POI = [self.global_goal_x, self.global_goal_y]
        
        self.delete_nodes()
        
        return POI
        
##############################################################
    def calc_lidar_coordinate(self, index, dist, pos_x, pos_y):
        if index > 3.5:
            index = 3.5
        x_cordinate = dist * -math.cos(math.radians(index)) + pos_x
        y_cordinate = dist * -math.sin(math.radians(index)) + pos_y
        
        return x_cordinate, y_cordinate
    
    def delete_nodes(self):
        while len(self.node) > 30:
            self.node.popleft()
        
    def check_dup_node(self):
        pass
    
    def heuristic(self, pos_x, pos_y):
        ########################################################
        # TODO
        # Implement heuristic function with other fuctions below
        # returns POI fron candidate nodes
        ########################################################
        
        heuristic_scores = []
        for candidate in self.node:
            ds = self.dist(candidate[0], candidate[1], pos_x, pos_y)
            ged = self.global_euclidean_distance(candidate[0], candidate[1])
            mi = self.map_information(candidate[0], candidate[1])
            heuristic_scores.append(ds + ged + mi)
        
        return heuristic_scores
    
    ### l1 = 5, l2 = 10 (Constnt)
    def distance_score(self, candidate_x, candidate_y, pos_x, pos_y, l1 = 5, l2 = 10):
        numerator = math.e ** ((math.sqrt((candidate_x - pos_x) ** 2 + (candidate_y - pos_y) ** 2) / (l2 - l1)) ** 2)
        
        denominator = math.e ** ((l2/(l2-l1)) ** 2)
        
        tem = math.tanh(numerator / denominator) * l2
        
        return tem
        
    def global_euclidean_distance(self, candidate_x, candidate_y):
        return math.sqrt((candidate_x - self.global_goal_x) ** 2 + (candidate_y - self.global_goal_y) ** 2)
        
    def dist(self, x1, y1, x2, y2):
        return math.sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2)
        
        #### Later?? #### (Not implemented yet)
    def map_information(self, candidate_x, candidate_y, k = 5):
        return 0
        sum1 = 0
        ########################################################
        # TODO
        # Get Map
        ########################################################
        # Example Code
        data = readpgm('./map2.pgm')
        
        d = np.reshape(data[0],data[1])
        m_value = [list(item) for item in d]
        ########################################################
        
        d[d < 50] = 5 # obstacle
        d[d >= 253] = 1 # free
        d[d >= 50] = 0 # unknown
        ########################################################
        
        
        ########################################################
        # TODO
        # Match lidar sensor with Map
        ########################################################
        
        for w in range(-int(k/2), int(k/2)):
            for h in range(-int(k/2), int(k/2)):
                sum1 = d[candidate_x + w][candidate_y + h]
                
        tem = sum1 / (k**2)
        
        return math.e ** tem
        
def timeout_handler(num, stack):
    logger = logging.getLogger(__name__)
    logger.error('Goal has not been reached within the given timeout.')
    raise Exception("GOALTIMEOUT")
    
def real_goal(pos_x, pos_y):
    timeout = 10
#    rclpy.init(args=None)
    
    pose = [float(pos_x), float(pos_y)]
    
    print('Goal coordinate : ', pose)
    print()
    
    real_goal = RealGoal(pose)
    
    signal.signal(signal.SIGALRM, timeout_handler)
    signal.alarm(timeout)
    
    try:
        rclpy.spin(real_goal)
    except Exception as ex:
        if "GOALTIMEOUT" in ex:
            print('Timeout time : ', timeout)
    finally:
        signal.alarm(0)

        return True

def main(args=sys.argv[1], args1=sys.argv[2]):
    print("Start test codes")
    pose = [float(args), float(args1)]
    rclpy.init(args=None)
    sub = SensorSub(pose)
    Env = ImplementEnv(pose)
    while Env.is_in_process:
        print(Env.node)
        for _ in range(10):
            rclpy.spin_once(sub)

        poi = Env.step(sub.scanMsg.ranges, sub.last_pose_x, sub.last_pose_y)
        
        real_goal(poi[0], poi[1])

    rclpy.shutdown()

if __name__ == '__main__':
    main()

