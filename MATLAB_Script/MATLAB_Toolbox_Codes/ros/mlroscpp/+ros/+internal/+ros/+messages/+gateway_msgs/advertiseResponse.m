function [data, info] = advertiseResponse
%Advertise gives an empty data for gateway_msgs/AdvertiseResponse

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'gateway_msgs/AdvertiseResponse';
[data.Result, info.Result] = ros.internal.ros.messages.ros.default_type('int8',1);
[data.ErrorMessage, info.ErrorMessage] = ros.internal.ros.messages.ros.char('string',0);
[data.Watchlist, info.Watchlist] = ros.internal.ros.messages.gateway_msgs.rule;
info.Watchlist.MLdataType = 'struct';
info.Watchlist.MaxLen = NaN;
info.Watchlist.MinLen = 0;
data.Watchlist = data.Watchlist([],1);
info.MessageType = 'gateway_msgs/AdvertiseResponse';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,6);
info.MatPath{1} = 'result';
info.MatPath{2} = 'error_message';
info.MatPath{3} = 'watchlist';
info.MatPath{4} = 'watchlist.type';
info.MatPath{5} = 'watchlist.name';
info.MatPath{6} = 'watchlist.node';
