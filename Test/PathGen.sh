#!/bin/sh

echo "Generating Path with MATLAB\n"
/usr/local/MATLAB/R2022b/bin/matlab -nodisplay -r "cd('/home/turtlebot/Desktop/MATLAB_Scripts/SLAM'); RRT('./Maps/map.pgm', $1, $2, $3, $4);exit"
# matlab -nodisplay -r "cd('/home/turtlebot/Desktop/MATLAB_Scripts/SLAM'); SLAM_Node($1);exit"
echo "Process has done!"
