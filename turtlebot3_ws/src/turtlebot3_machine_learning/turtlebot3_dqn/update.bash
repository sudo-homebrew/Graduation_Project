#! /bin/bash

colcon build --symlink-install
. ~/turtlebot3_ws/install/local_setup.bash
source /opt/ros/foxy/setup.bash && source ~/turtlebot3_ws/install/local_setup.bash
source install/setup.bash

