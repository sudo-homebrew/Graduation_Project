function [data, info] = jointCommand
%JointCommand gives an empty data for pendulum_msgs/JointCommand

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'pendulum_msgs/JointCommand';
[data.position, info.position] = ros.internal.ros2.messages.ros2.default_type('double',1,0);
info.MessageType = 'pendulum_msgs/JointCommand';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,1);
info.MatPath{1} = 'position';
