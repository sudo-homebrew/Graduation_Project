function [data, info] = led
%Led gives an empty data for kobuki_msgs/Led

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'kobuki_msgs/Led';
[data.BLACK, info.BLACK] = ros.internal.ros.messages.ros.default_type('uint8',1, 0);
[data.GREEN, info.GREEN] = ros.internal.ros.messages.ros.default_type('uint8',1, 1);
[data.ORANGE, info.ORANGE] = ros.internal.ros.messages.ros.default_type('uint8',1, 2);
[data.RED, info.RED] = ros.internal.ros.messages.ros.default_type('uint8',1, 3);
[data.Value, info.Value] = ros.internal.ros.messages.ros.default_type('uint8',1);
info.MessageType = 'kobuki_msgs/Led';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,5);
info.MatPath{1} = 'BLACK';
info.MatPath{2} = 'GREEN';
info.MatPath{3} = 'ORANGE';
info.MatPath{4} = 'RED';
info.MatPath{5} = 'value';
