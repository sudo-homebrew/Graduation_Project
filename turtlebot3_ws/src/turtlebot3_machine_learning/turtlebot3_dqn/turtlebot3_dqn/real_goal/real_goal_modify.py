#!/usr/bin/env python3
#

import os
import signal
import random
import sys
import subprocess
import time

from gazebo_msgs.srv import DeleteEntity
from gazebo_msgs.srv import SpawnEntity
from geometry_msgs.msg import Pose
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from std_srvs.srv import Empty
# pip install pandas
import pandas as pd

class RealGoal(Node):
    def __init__(self, pose):
        super().__init__('real_goal')

        """************************************************************
        ** Initialise variables
        ************************************************************"""
        self.length = len(pose) # pose 개수
        self.fx = pose[0][0]
        self.fy = pose[0][1]

        self.pose = pose
        self.goal_pose_x = 0
        self.goal_pose_y = 0


        # self.goal_pose_x = pose[0]
        # self.goal_pose_y = pose[1]

        self.init_state = False

        """************************************************************
        ** Initialise ROS publishers, subscribers and clients
        ************************************************************"""
        qos = QoSProfile(depth=10)

        # Initialise publishers
        self.goal_pose_pub = self.create_publisher(Pose, 'goal_pose', qos)

        # Initialise client

        # Initialise servers
        self.task_succeed_server = self.create_service(
            Empty,
            'task_succeed',
            self.task_succeed_callback)
        self.task_fail_server = self.create_service(Empty, 'task_fail', self.task_fail_callback)


        # Process
        self.publish_timer = self.create_timer(
            0.010,  # unit: s
            self.publish_callback)

    """*******************************************************************************
    ** Callback functions and relevant functions
    *******************************************************************************"""

    def publish_callback(self):
        # self.reset_simulation()
        # Init
        if self.init_state is False:
            # self.delete_entity()
            # self.reset_simulation()
            self.init_state = True
            print("init!!!")
            print("Goal pose: ", self.goal_pose_x, self.goal_pose_y)

        # Publish goal pose
        goal_pose = Pose()
        goal_pose.position.x = self.goal_pose_x
        goal_pose.position.y = self.goal_pose_y
        self.goal_pose_pub.publish(goal_pose)
        #self.spawn_entity()

    def task_succeed_callback(self, request, response):
        #self.delete_entity()
        self.generate_goal_pose()
        print("generate a new goal :)")

        return response

    def task_fail_callback(self, request, response):
        # self.delete_entity()
        # self.reset_simulation()
        #self.generate_goal_pose()
        print("fail!")
        #print("reset the gazebo environment :(")
        # finishtrajectory
        # self.finish_trajectory()
        # self.start_trajectory()

        return response

    def generate_goal_pose(self):
        for i in range(self.length):
            #x, y = input("input goal pose : ").split(" ")
            self.goal_pose_x = float(self.pose[i][0]-self.fx)/100 # meter
            self.goal_pose_y = float(self.pose[i][1]-self.fy)/100 # meter
            print("Goal pose: ", self.goal_pose_x, self.goal_pose_y)
#def main(args=None):
#    rclpy.init(args=args)
#    real_goal = RealGoal(args)
#    rclpy.spin(real_goal)
#
#    real_goal.destroy()
#    rclpy.shutdown()

def main(args=sys.argv[1], args1=sys.argv[2]):
    rclpy.init(args=args)
    #print(args)
    #print(args1)
    path = pd.read_csv('path.csv', header=None)
    #pose = [float(args), float(args1)]
    pose = path.values
    #print(pose)
    real_goal = RealGoal(pose)
    rclpy.spin(real_goal)

    real_goal.destroy()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
