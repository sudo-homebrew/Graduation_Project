function [data, info] = connectHubRequest
%ConnectHub gives an empty data for gateway_msgs/ConnectHubRequest

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'gateway_msgs/ConnectHubRequest';
[data.Uri, info.Uri] = ros.internal.ros.messages.ros.char('string',0);
info.MessageType = 'gateway_msgs/ConnectHubRequest';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,1);
info.MatPath{1} = 'uri';
