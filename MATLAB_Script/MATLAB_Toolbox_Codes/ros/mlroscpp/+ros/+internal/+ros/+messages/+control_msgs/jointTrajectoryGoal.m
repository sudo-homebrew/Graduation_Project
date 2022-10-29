function [data, info] = jointTrajectoryGoal
%JointTrajectoryGoal gives an empty data for control_msgs/JointTrajectoryGoal

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'control_msgs/JointTrajectoryGoal';
[data.Trajectory, info.Trajectory] = ros.internal.ros.messages.trajectory_msgs.jointTrajectory;
info.Trajectory.MLdataType = 'struct';
info.MessageType = 'control_msgs/JointTrajectoryGoal';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,16);
info.MatPath{1} = 'trajectory';
info.MatPath{2} = 'trajectory.header';
info.MatPath{3} = 'trajectory.header.seq';
info.MatPath{4} = 'trajectory.header.stamp';
info.MatPath{5} = 'trajectory.header.stamp.sec';
info.MatPath{6} = 'trajectory.header.stamp.nsec';
info.MatPath{7} = 'trajectory.header.frame_id';
info.MatPath{8} = 'trajectory.joint_names';
info.MatPath{9} = 'trajectory.points';
info.MatPath{10} = 'trajectory.points.positions';
info.MatPath{11} = 'trajectory.points.velocities';
info.MatPath{12} = 'trajectory.points.accelerations';
info.MatPath{13} = 'trajectory.points.effort';
info.MatPath{14} = 'trajectory.points.time_from_start';
info.MatPath{15} = 'trajectory.points.time_from_start.sec';
info.MatPath{16} = 'trajectory.points.time_from_start.nsec';
