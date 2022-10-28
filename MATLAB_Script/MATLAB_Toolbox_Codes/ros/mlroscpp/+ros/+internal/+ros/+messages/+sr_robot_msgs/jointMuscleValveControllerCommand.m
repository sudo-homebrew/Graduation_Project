function [data, info] = jointMuscleValveControllerCommand
%JointMuscleValveControllerCommand gives an empty data for sr_robot_msgs/JointMuscleValveControllerCommand

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'sr_robot_msgs/JointMuscleValveControllerCommand';
[data.CmdValveMuscle, info.CmdValveMuscle] = ros.internal.ros.messages.ros.default_type('int8',2);
[data.CmdDurationMs, info.CmdDurationMs] = ros.internal.ros.messages.ros.default_type('uint64',2);
info.MessageType = 'sr_robot_msgs/JointMuscleValveControllerCommand';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,2);
info.MatPath{1} = 'cmd_valve_muscle';
info.MatPath{2} = 'cmd_duration_ms';
