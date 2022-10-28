function [data, info] = jointState
%JointState gives an empty data for pendulum_msgs/JointState

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'pendulum_msgs/JointState';
[data.position, info.position] = ros.internal.ros2.messages.ros2.default_type('double',1,0);
[data.velocity, info.velocity] = ros.internal.ros2.messages.ros2.default_type('double',1,0);
[data.effort, info.effort] = ros.internal.ros2.messages.ros2.default_type('double',1,0);
info.MessageType = 'pendulum_msgs/JointState';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,3);
info.MatPath{1} = 'position';
info.MatPath{2} = 'velocity';
info.MatPath{3} = 'effort';
