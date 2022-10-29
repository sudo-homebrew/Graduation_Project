function [data, info] = remoteRule
%RemoteRule gives an empty data for gateway_msgs/RemoteRule

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'gateway_msgs/RemoteRule';
[data.Gateway, info.Gateway] = ros.internal.ros.messages.ros.char('string',0);
[data.Rule, info.Rule] = ros.internal.ros.messages.gateway_msgs.rule;
info.Rule.MLdataType = 'struct';
info.MessageType = 'gateway_msgs/RemoteRule';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,5);
info.MatPath{1} = 'gateway';
info.MatPath{2} = 'rule';
info.MatPath{3} = 'rule.type';
info.MatPath{4} = 'rule.name';
info.MatPath{5} = 'rule.node';
