function [data, info] = digitalOutput
%DigitalOutput gives an empty data for kobuki_msgs/DigitalOutput

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'kobuki_msgs/DigitalOutput';
[data.Values, info.Values] = ros.internal.ros.messages.ros.default_type('logical',4);
[data.Mask, info.Mask] = ros.internal.ros.messages.ros.default_type('logical',4);
info.MessageType = 'kobuki_msgs/DigitalOutput';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,2);
info.MatPath{1} = 'values';
info.MatPath{2} = 'mask';
