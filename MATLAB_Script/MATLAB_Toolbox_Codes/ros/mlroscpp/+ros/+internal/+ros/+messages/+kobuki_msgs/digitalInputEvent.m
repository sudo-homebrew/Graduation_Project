function [data, info] = digitalInputEvent
%DigitalInputEvent gives an empty data for kobuki_msgs/DigitalInputEvent

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'kobuki_msgs/DigitalInputEvent';
[data.Values, info.Values] = ros.internal.ros.messages.ros.default_type('logical',4);
info.MessageType = 'kobuki_msgs/DigitalInputEvent';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,1);
info.MatPath{1} = 'values';
