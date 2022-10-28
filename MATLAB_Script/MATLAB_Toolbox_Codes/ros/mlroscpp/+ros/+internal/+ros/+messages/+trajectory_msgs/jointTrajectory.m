function [data, info] = jointTrajectory
%JointTrajectory gives an empty data for trajectory_msgs/JointTrajectory

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'trajectory_msgs/JointTrajectory';
[data.Header, info.Header] = ros.internal.ros.messages.std_msgs.header;
info.Header.MLdataType = 'struct';
[data.JointNames, info.JointNames] = ros.internal.ros.messages.ros.char('string',NaN);
[data.Points, info.Points] = ros.internal.ros.messages.trajectory_msgs.jointTrajectoryPoint;
info.Points.MLdataType = 'struct';
info.Points.MaxLen = NaN;
info.Points.MinLen = 0;
data.Points = data.Points([],1);
info.MessageType = 'trajectory_msgs/JointTrajectory';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,15);
info.MatPath{1} = 'header';
info.MatPath{2} = 'header.seq';
info.MatPath{3} = 'header.stamp';
info.MatPath{4} = 'header.stamp.sec';
info.MatPath{5} = 'header.stamp.nsec';
info.MatPath{6} = 'header.frame_id';
info.MatPath{7} = 'joint_names';
info.MatPath{8} = 'points';
info.MatPath{9} = 'points.positions';
info.MatPath{10} = 'points.velocities';
info.MatPath{11} = 'points.accelerations';
info.MatPath{12} = 'points.effort';
info.MatPath{13} = 'points.time_from_start';
info.MatPath{14} = 'points.time_from_start.sec';
info.MatPath{15} = 'points.time_from_start.nsec';
