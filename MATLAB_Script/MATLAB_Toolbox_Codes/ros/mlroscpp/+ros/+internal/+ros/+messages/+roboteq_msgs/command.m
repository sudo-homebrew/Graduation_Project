function [data, info] = command
%Command gives an empty data for roboteq_msgs/Command

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'roboteq_msgs/Command';
[data.MODESTOPPED, info.MODESTOPPED] = ros.internal.ros.messages.ros.default_type('int8',1, -1);
[data.MODEVELOCITY, info.MODEVELOCITY] = ros.internal.ros.messages.ros.default_type('int8',1, 0);
[data.MODEPOSITION, info.MODEPOSITION] = ros.internal.ros.messages.ros.default_type('int8',1, 1);
[data.Mode, info.Mode] = ros.internal.ros.messages.ros.default_type('int8',1);
[data.Setpoint, info.Setpoint] = ros.internal.ros.messages.ros.default_type('single',1);
info.MessageType = 'roboteq_msgs/Command';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,5);
info.MatPath{1} = 'MODE_STOPPED';
info.MatPath{2} = 'MODE_VELOCITY';
info.MatPath{3} = 'MODE_POSITION';
info.MatPath{4} = 'mode';
info.MatPath{5} = 'setpoint';
