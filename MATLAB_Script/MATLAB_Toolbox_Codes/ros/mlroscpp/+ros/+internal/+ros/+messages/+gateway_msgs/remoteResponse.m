function [data, info] = remoteResponse
%Remote gives an empty data for gateway_msgs/RemoteResponse

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'gateway_msgs/RemoteResponse';
[data.Result, info.Result] = ros.internal.ros.messages.ros.default_type('int8',1);
[data.ErrorMessage, info.ErrorMessage] = ros.internal.ros.messages.ros.char('string',0);
info.MessageType = 'gateway_msgs/RemoteResponse';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,2);
info.MatPath{1} = 'result';
info.MatPath{2} = 'error_message';
