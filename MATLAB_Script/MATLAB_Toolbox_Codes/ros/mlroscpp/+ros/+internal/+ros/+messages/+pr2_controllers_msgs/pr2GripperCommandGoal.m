function [data, info] = pr2GripperCommandGoal
%Pr2GripperCommandGoal gives an empty data for pr2_controllers_msgs/Pr2GripperCommandGoal

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'pr2_controllers_msgs/Pr2GripperCommandGoal';
[data.Command, info.Command] = ros.internal.ros.messages.pr2_controllers_msgs.pr2GripperCommand;
info.Command.MLdataType = 'struct';
info.MessageType = 'pr2_controllers_msgs/Pr2GripperCommandGoal';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,3);
info.MatPath{1} = 'command';
info.MatPath{2} = 'command.position';
info.MatPath{3} = 'command.max_effort';
