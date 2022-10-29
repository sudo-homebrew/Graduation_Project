function [data, info] = advertiseAllResponse
%AdvertiseAll gives an empty data for gateway_msgs/AdvertiseAllResponse

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'gateway_msgs/AdvertiseAllResponse';
[data.Result, info.Result] = ros.internal.ros.messages.ros.default_type('int8',1);
[data.ErrorMessage, info.ErrorMessage] = ros.internal.ros.messages.ros.char('string',0);
[data.Blacklist, info.Blacklist] = ros.internal.ros.messages.gateway_msgs.rule;
info.Blacklist.MLdataType = 'struct';
info.Blacklist.MaxLen = NaN;
info.Blacklist.MinLen = 0;
data.Blacklist = data.Blacklist([],1);
info.MessageType = 'gateway_msgs/AdvertiseAllResponse';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,6);
info.MatPath{1} = 'result';
info.MatPath{2} = 'error_message';
info.MatPath{3} = 'blacklist';
info.MatPath{4} = 'blacklist.type';
info.MatPath{5} = 'blacklist.name';
info.MatPath{6} = 'blacklist.node';
