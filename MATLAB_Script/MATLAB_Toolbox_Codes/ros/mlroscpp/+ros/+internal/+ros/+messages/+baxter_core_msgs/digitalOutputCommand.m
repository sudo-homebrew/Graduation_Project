function [data, info] = digitalOutputCommand
%DigitalOutputCommand gives an empty data for baxter_core_msgs/DigitalOutputCommand

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'baxter_core_msgs/DigitalOutputCommand';
[data.Name, info.Name] = ros.internal.ros.messages.ros.char('string',0);
[data.Value, info.Value] = ros.internal.ros.messages.ros.default_type('logical',1);
info.MessageType = 'baxter_core_msgs/DigitalOutputCommand';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,2);
info.MatPath{1} = 'name';
info.MatPath{2} = 'value';
