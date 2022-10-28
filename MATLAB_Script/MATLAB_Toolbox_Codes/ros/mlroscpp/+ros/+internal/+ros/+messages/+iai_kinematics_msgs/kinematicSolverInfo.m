function [data, info] = kinematicSolverInfo
%KinematicSolverInfo gives an empty data for iai_kinematics_msgs/KinematicSolverInfo

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'iai_kinematics_msgs/KinematicSolverInfo';
[data.JointNames, info.JointNames] = ros.internal.ros.messages.ros.char('string',NaN);
[data.Limits, info.Limits] = ros.internal.ros.messages.iai_kinematics_msgs.jointLimits;
info.Limits.MLdataType = 'struct';
info.Limits.MaxLen = NaN;
info.Limits.MinLen = 0;
data.Limits = data.Limits([],1);
[data.LinkNames, info.LinkNames] = ros.internal.ros.messages.ros.char('string',NaN);
info.MessageType = 'iai_kinematics_msgs/KinematicSolverInfo';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,11);
info.MatPath{1} = 'joint_names';
info.MatPath{2} = 'limits';
info.MatPath{3} = 'limits.joint_name';
info.MatPath{4} = 'limits.has_position_limits';
info.MatPath{5} = 'limits.min_position';
info.MatPath{6} = 'limits.max_position';
info.MatPath{7} = 'limits.has_velocity_limits';
info.MatPath{8} = 'limits.max_velocity';
info.MatPath{9} = 'limits.has_acceleration_limits';
info.MatPath{10} = 'limits.max_acceleration';
info.MatPath{11} = 'link_names';
