function [data, info] = advertiseAllRequest
%AdvertiseAll gives an empty data for gateway_msgs/AdvertiseAllRequest

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'gateway_msgs/AdvertiseAllRequest';
[data.Cancel, info.Cancel] = ros.internal.ros.messages.ros.default_type('logical',1);
[data.Blacklist, info.Blacklist] = ros.internal.ros.messages.gateway_msgs.rule;
info.Blacklist.MLdataType = 'struct';
info.Blacklist.MaxLen = NaN;
info.Blacklist.MinLen = 0;
data.Blacklist = data.Blacklist([],1);
info.MessageType = 'gateway_msgs/AdvertiseAllRequest';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,5);
info.MatPath{1} = 'cancel';
info.MatPath{2} = 'blacklist';
info.MatPath{3} = 'blacklist.type';
info.MatPath{4} = 'blacklist.name';
info.MatPath{5} = 'blacklist.node';
