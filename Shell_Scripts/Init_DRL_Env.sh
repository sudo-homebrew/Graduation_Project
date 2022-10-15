#!/bin/bash

#This shell script is for run Gazebo World with Dynamic Obstacles2

#ROS2 environment setting
source /opt/ros/foxy/setup.bash && source ~/turtlebot3_ws/install/local_setup.bash

#Run ROS2 train helper
ros2 run turtlebot3_dqn dqn_gazebo 4
