function [data, info] = jointTrajectoryPoint
%JointTrajectoryPoint gives an empty data for trajectory_msgs/JointTrajectoryPoint

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'trajectory_msgs/JointTrajectoryPoint';
[data.Positions, info.Positions] = ros.internal.ros.messages.ros.default_type('double',NaN);
[data.Velocities, info.Velocities] = ros.internal.ros.messages.ros.default_type('double',NaN);
[data.Accelerations, info.Accelerations] = ros.internal.ros.messages.ros.default_type('double',NaN);
[data.Effort, info.Effort] = ros.internal.ros.messages.ros.default_type('double',NaN);
[data.TimeFromStart, info.TimeFromStart] = ros.internal.ros.messages.ros.duration;
info.TimeFromStart.MLdataType = 'struct';
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
info.MatPath{7} = 'time_from_start.nsec';
