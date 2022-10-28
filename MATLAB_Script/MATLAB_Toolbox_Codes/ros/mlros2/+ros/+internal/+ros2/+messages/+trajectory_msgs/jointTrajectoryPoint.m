function [data, info] = jointTrajectoryPoint
%JointTrajectoryPoint gives an empty data for trajectory_msgs/JointTrajectoryPoint

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'trajectory_msgs/JointTrajectoryPoint';
[data.positions, info.positions] = ros.internal.ros2.messages.ros2.default_type('double',NaN,0);
[data.velocities, info.velocities] = ros.internal.ros2.messages.ros2.default_type('double',NaN,0);
[data.accelerations, info.accelerations] = ros.internal.ros2.messages.ros2.default_type('double',NaN,0);
[data.effort, info.effort] = ros.internal.ros2.messages.ros2.default_type('double',NaN,0);
[data.time_from_start, info.time_from_start] = ros.internal.ros2.messages.builtin_interfaces.duration;
info.time_from_start.MLdataType = 'struct';
info.MessageType = 'trajectory_msgs/JointTrajectoryPoint';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,7);
info.MatPath{1} = 'positions';
info.MatPath{2} = 'velocities';
info.MatPath{3} = 'accelerations';
info.MatPath{4} = 'effort';
info.MatPath{5} = 'time_from_start';
info.MatPath{6} = 'time_from_start.sec';
info.MatPath{7} = 'time_from_start.nanosec';
