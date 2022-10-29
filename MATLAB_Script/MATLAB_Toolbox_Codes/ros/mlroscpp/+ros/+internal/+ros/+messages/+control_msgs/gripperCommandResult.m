function [data, info] = gripperCommandResult
%GripperCommandResult gives an empty data for control_msgs/GripperCommandResult

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'control_msgs/GripperCommandResult';
[data.Position, info.Position] = ros.internal.ros.messages.ros.default_type('double',1);
[data.Effort, info.Effort] = ros.internal.ros.messages.ros.default_type('double',1);
[data.Stalled, info.Stalled] = ros.internal.ros.messages.ros.default_type('logical',1);
[data.ReachedGoal, info.ReachedGoal] = ros.internal.ros.messages.ros.default_type('logical',1);
info.MessageType = 'control_msgs/GripperCommandResult';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,4);
info.MatPath{1} = 'position';
info.MatPath{2} = 'effort';
info.MatPath{3} = 'stalled';
info.MatPath{4} = 'reached_goal';
