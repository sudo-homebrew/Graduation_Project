function [data, info] = sendCommandResponse
%SendCommand gives an empty data for nav2d_navigator/SendCommandResponse

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'nav2d_navigator/SendCommandResponse';
[data.Response, info.Response] = ros.internal.ros.messages.ros.default_type('int8',1);
info.MessageType = 'nav2d_navigator/SendCommandResponse';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,1);
info.MatPath{1} = 'response';
