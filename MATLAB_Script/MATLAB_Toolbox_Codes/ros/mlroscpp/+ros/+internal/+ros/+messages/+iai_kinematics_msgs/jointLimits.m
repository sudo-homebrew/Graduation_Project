function [data, info] = jointLimits
%JointLimits gives an empty data for iai_kinematics_msgs/JointLimits

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'iai_kinematics_msgs/JointLimits';
[data.JointName, info.JointName] = ros.internal.ros.messages.ros.char('string',0);
[data.HasPositionLimits, info.HasPositionLimits] = ros.internal.ros.messages.ros.default_type('logical',1);
[data.MinPosition, info.MinPosition] = ros.internal.ros.messages.ros.default_type('double',1);
[data.MaxPosition, info.MaxPosition] = ros.internal.ros.messages.ros.default_type('double',1);
[data.HasVelocityLimits, info.HasVelocityLimits] = ros.internal.ros.messages.ros.default_type('logical',1);
[data.MaxVelocity, info.MaxVelocity] = ros.internal.ros.messages.ros.default_type('double',1);
[data.HasAccelerationLimits, info.HasAccelerationLimits] = ros.internal.ros.messages.ros.default_type('logical',1);
[data.MaxAcceleration, info.MaxAcceleration] = ros.internal.ros.messages.ros.default_type('double',1);
info.MessageType = 'iai_kinematics_msgs/JointLimits';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,8);
info.MatPath{1} = 'joint_name';
info.MatPath{2} = 'has_position_limits';
info.MatPath{3} = 'min_position';
info.MatPath{4} = 'max_position';
info.MatPath{5} = 'has_velocity_limits';
info.MatPath{6} = 'max_velocity';
info.MatPath{7} = 'has_acceleration_limits';
info.MatPath{8} = 'max_acceleration';
