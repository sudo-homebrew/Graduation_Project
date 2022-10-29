function [data, info] = string
%String gives an empty data for std_msgs/String

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'std_msgs/String';
[data.data, info.data] = ros.internal.ros2.messages.ros2.char('string',1,NaN,0);
info.MessageType = 'std_msgs/String';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,1);
info.MatPath{1} = 'data';
