function [data, info] = jointTrajectory
%JointTrajectory gives an empty data for trajectory_msgs/JointTrajectory

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'trajectory_msgs/JointTrajectory';
[data.header, info.header] = ros.internal.ros2.messages.std_msgs.header;
info.header.MLdataType = 'struct';
[data.joint_names, info.joint_names] = ros.internal.ros2.messages.ros2.char('string',NaN,NaN,0);
[data.points, info.points] = ros.internal.ros2.messages.trajectory_msgs.jointTrajectoryPoint;
info.points.MLdataType = 'struct';
info.points.MaxLen = NaN;
info.points.MinLen = 0;
info.MessageType = 'trajectory_msgs/JointTrajectory';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,14);
info.MatPath{1} = 'header';
info.MatPath{2} = 'header.stamp';
info.MatPath{3} = 'header.stamp.sec';
info.MatPath{4} = 'header.stamp.nanosec';
info.MatPath{5} = 'header.frame_id';
info.MatPath{6} = 'joint_names';
info.MatPath{7} = 'points';
info.MatPath{8} = 'points.positions';
info.MatPath{9} = 'points.velocities';
info.MatPath{10} = 'points.accelerations';
info.MatPath{11} = 'points.effort';
info.MatPath{12} = 'points.time_from_start';
info.MatPath{13} = 'points.time_from_start.sec';
info.MatPath{14} = 'points.time_from_start.nanosec';
