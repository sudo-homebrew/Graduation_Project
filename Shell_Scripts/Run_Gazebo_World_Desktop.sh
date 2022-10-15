#!/bin/bash

#This shell script is for run Gazebo World with Dynamic Obstacles

#Setting for Gazebo Graphica
#No need for desktop version
#export GAZEBO_IP=127.0.0.1
#export DISPLAY=$(cat /etc/resolv.conf | grep nameserver | awk '{print $2}'):0
#export LIBGL_ALWAYS_INDIRECT=0

#Setting for ROS2 environment
source /opt/ros/foxy/setup.bash && source ~/turtlebot3_ws/install/local_setup.bash

#ROS2 Dynamic Obstacle setting
export GAZEBO_PLUGIN_PATH=$HOME/turtlebot3_ws/build/turtlebot3_gazebo:$GAZEBO_PLUGIN_PATH

#Confocire Turtlebot3 model
export TURTLEBOT3_MODEL=waffle_pi

#Launching Map
ros2 launch turtlebot3_gazebo turtlebot3_dqn_stage4.launch.py
