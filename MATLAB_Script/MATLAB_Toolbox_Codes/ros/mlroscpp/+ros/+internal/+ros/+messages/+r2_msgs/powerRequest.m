function [data, info] = powerRequest
%Power gives an empty data for r2_msgs/PowerRequest

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'r2_msgs/PowerRequest';
[data.Channel, info.Channel] = ros.internal.ros.messages.ros.char('string',0);
[data.State, info.State] = ros.internal.ros.messages.ros.default_type('logical',1);
info.MessageType = 'r2_msgs/PowerRequest';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,2);
info.MatPath{1} = 'channel';
info.MatPath{2} = 'state';
