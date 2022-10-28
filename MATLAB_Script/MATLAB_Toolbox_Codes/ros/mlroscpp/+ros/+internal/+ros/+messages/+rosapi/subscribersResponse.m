function [data, info] = subscribersResponse
%Subscribers gives an empty data for rosapi/SubscribersResponse

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'rosapi/SubscribersResponse';
[data.Subscribers, info.Subscribers] = ros.internal.ros.messages.ros.char('string',NaN);
info.MessageType = 'rosapi/SubscribersResponse';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,1);
info.MatPath{1} = 'subscribers';
