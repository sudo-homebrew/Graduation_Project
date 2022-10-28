function [data, info] = remoteAllRequest
%RemoteAll gives an empty data for gateway_msgs/RemoteAllRequest

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'gateway_msgs/RemoteAllRequest';
[data.Gateway, info.Gateway] = ros.internal.ros.messages.ros.char('string',0);
[data.Blacklist, info.Blacklist] = ros.internal.ros.messages.gateway_msgs.rule;
info.Blacklist.MLdataType = 'struct';
info.Blacklist.MaxLen = NaN;
info.Blacklist.MinLen = 0;
data.Blacklist = data.Blacklist([],1);
[data.Cancel, info.Cancel] = ros.internal.ros.messages.ros.default_type('logical',1);
info.MessageType = 'gateway_msgs/RemoteAllRequest';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,6);
info.MatPath{1} = 'gateway';
info.MatPath{2} = 'blacklist';
info.MatPath{3} = 'blacklist.type';
info.MatPath{4} = 'blacklist.name';
info.MatPath{5} = 'blacklist.node';
info.MatPath{6} = 'cancel';
