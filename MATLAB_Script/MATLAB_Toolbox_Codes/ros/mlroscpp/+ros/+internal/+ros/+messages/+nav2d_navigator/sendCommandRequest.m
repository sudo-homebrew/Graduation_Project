function [data, info] = sendCommandRequest
%SendCommand gives an empty data for nav2d_navigator/SendCommandRequest

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'nav2d_navigator/SendCommandRequest';
[data.Command, info.Command] = ros.internal.ros.messages.ros.default_type('int8',1);
info.MessageType = 'nav2d_navigator/SendCommandRequest';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,1);
info.MatPath{1} = 'command';
