function [data, info] = pr2GripperCommand
%Pr2GripperCommand gives an empty data for pr2_controllers_msgs/Pr2GripperCommand

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'pr2_controllers_msgs/Pr2GripperCommand';
[data.Position, info.Position] = ros.internal.ros.messages.ros.default_type('double',1);
[data.MaxEffort, info.MaxEffort] = ros.internal.ros.messages.ros.default_type('double',1);
info.MessageType = 'pr2_controllers_msgs/Pr2GripperCommand';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,2);
info.MatPath{1} = 'position';
info.MatPath{2} = 'max_effort';
