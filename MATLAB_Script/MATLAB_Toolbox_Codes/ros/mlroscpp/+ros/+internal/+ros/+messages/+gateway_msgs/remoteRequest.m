function [data, info] = remoteRequest
%Remote gives an empty data for gateway_msgs/RemoteRequest

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'gateway_msgs/RemoteRequest';
[data.Remotes, info.Remotes] = ros.internal.ros.messages.gateway_msgs.remoteRule;
info.Remotes.MLdataType = 'struct';
info.Remotes.MaxLen = NaN;
info.Remotes.MinLen = 0;
data.Remotes = data.Remotes([],1);
[data.Cancel, info.Cancel] = ros.internal.ros.messages.ros.default_type('logical',1);
info.MessageType = 'gateway_msgs/RemoteRequest';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,7);
info.MatPath{1} = 'remotes';
info.MatPath{2} = 'remotes.gateway';
info.MatPath{3} = 'remotes.rule';
info.MatPath{4} = 'remotes.rule.type';
info.MatPath{5} = 'remotes.rule.name';
info.MatPath{6} = 'remotes.rule.node';
info.MatPath{7} = 'cancel';
