function [data, info] = multiDOFJointTrajectory
%MultiDOFJointTrajectory gives an empty data for trajectory_msgs/MultiDOFJointTrajectory

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'trajectory_msgs/MultiDOFJointTrajectory';
[data.Header, info.Header] = ros.internal.ros.messages.std_msgs.header;
info.Header.MLdataType = 'struct';
[data.JointNames, info.JointNames] = ros.internal.ros.messages.ros.char('string',NaN);
[data.Points, info.Points] = ros.internal.ros.messages.trajectory_msgs.multiDOFJointTrajectoryPoint;
info.Points.MLdataType = 'struct';
info.Points.MaxLen = NaN;
info.Points.MinLen = 0;
data.Points = data.Points([],1);
info.MessageType = 'trajectory_msgs/MultiDOFJointTrajectory';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,39);
info.MatPath{1} = 'header';
info.MatPath{2} = 'header.seq';
info.MatPath{3} = 'header.stamp';
info.MatPath{4} = 'header.stamp.sec';
info.MatPath{5} = 'header.stamp.nsec';
info.MatPath{6} = 'header.frame_id';
info.MatPath{7} = 'joint_names';
info.MatPath{8} = 'points';
info.MatPath{9} = 'points.transforms';
info.MatPath{10} = 'points.transforms.translation';
info.MatPath{11} = 'points.transforms.translation.x';
info.MatPath{12} = 'points.transforms.translation.y';
info.MatPath{13} = 'points.transforms.translation.z';
info.MatPath{14} = 'points.transforms.rotation';
info.MatPath{15} = 'points.transforms.rotation.x';
info.MatPath{16} = 'points.transforms.rotation.y';
info.MatPath{17} = 'points.transforms.rotation.z';
info.MatPath{18} = 'points.transforms.rotation.w';
info.MatPath{19} = 'points.velocities';
info.MatPath{20} = 'points.velocities.linear';
info.MatPath{21} = 'points.velocities.linear.x';
info.MatPath{22} = 'points.velocities.linear.y';
info.MatPath{23} = 'points.velocities.linear.z';
info.MatPath{24} = 'points.velocities.angular';
info.MatPath{25} = 'points.velocities.angular.x';
info.MatPath{26} = 'points.velocities.angular.y';
info.MatPath{27} = 'points.velocities.angular.z';
info.MatPath{28} = 'points.accelerations';
info.MatPath{29} = 'points.accelerations.linear';
info.MatPath{30} = 'points.accelerations.linear.x';
info.MatPath{31} = 'points.accelerations.linear.y';
info.MatPath{32} = 'points.accelerations.linear.z';
info.MatPath{33} = 'points.accelerations.angular';
info.MatPath{34} = 'points.accelerations.angular.x';
info.MatPath{35} = 'points.accelerations.angular.y';
info.MatPath{36} = 'points.accelerations.angular.z';
info.MatPath{37} = 'points.time_from_start';
info.MatPath{38} = 'points.time_from_start.sec';
info.MatPath{39} = 'points.time_from_start.nsec';