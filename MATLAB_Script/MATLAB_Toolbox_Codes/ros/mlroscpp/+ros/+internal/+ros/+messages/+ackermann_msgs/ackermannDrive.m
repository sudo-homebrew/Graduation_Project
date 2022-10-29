function [data, info] = ackermannDrive
%AckermannDrive gives an empty data for ackermann_msgs/AckermannDrive

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'ackermann_msgs/AckermannDrive';
[data.SteeringAngle, info.SteeringAngle] = ros.internal.ros.messages.ros.default_type('single',1);
[data.SteeringAngleVelocity, info.SteeringAngleVelocity] = ros.internal.ros.messages.ros.default_type('single',1);
[data.Speed, info.Speed] = ros.internal.ros.messages.ros.default_type('single',1);
[data.Acceleration, info.Acceleration] = ros.internal.ros.messages.ros.default_type('single',1);
[data.Jerk, info.Jerk] = ros.internal.ros.messages.ros.default_type('single',1);
info.MessageType = 'ackermann_msgs/AckermannDrive';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,5);
info.MatPath{1} = 'steering_angle';
info.MatPath{2} = 'steering_angle_velocity';
info.MatPath{3} = 'speed';
info.MatPath{4} = 'acceleration';
info.MatPath{5} = 'jerk';
