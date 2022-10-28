function [data, info] = gripperCommand
%GripperCommand gives an empty data for control_msgs/GripperCommand

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'control_msgs/GripperCommand';
[data.Position, info.Position] = ros.internal.ros.messages.ros.default_type('double',1);
[data.MaxEffort, info.MaxEffort] = ros.internal.ros.messages.ros.default_type('double',1);
info.MessageType = 'control_msgs/GripperCommand';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,2);
info.MatPath{1} = 'position';
info.MatPath{2} = 'max_effort';
