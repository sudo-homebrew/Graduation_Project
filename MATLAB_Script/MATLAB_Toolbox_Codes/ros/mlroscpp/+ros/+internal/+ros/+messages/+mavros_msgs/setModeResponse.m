function [data, info] = setModeResponse
%SetMode gives an empty data for mavros_msgs/SetModeResponse

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'mavros_msgs/SetModeResponse';
[data.ModeSent, info.ModeSent] = ros.internal.ros.messages.ros.default_type('logical',1);
info.MessageType = 'mavros_msgs/SetModeResponse';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,1);
info.MatPath{1} = 'mode_sent';
