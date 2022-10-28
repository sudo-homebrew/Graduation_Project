function [data, info] = rule
%Rule gives an empty data for gateway_msgs/Rule

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'gateway_msgs/Rule';
[data.Type, info.Type] = ros.internal.ros.messages.ros.char('string',0);
[data.Name, info.Name] = ros.internal.ros.messages.ros.char('string',0);
[data.Node, info.Node] = ros.internal.ros.messages.ros.char('string',0);
info.MessageType = 'gateway_msgs/Rule';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,3);
info.MatPath{1} = 'type';
info.MatPath{2} = 'name';
info.MatPath{3} = 'node';
