function [data, info] = float32
%Float32 gives an empty data for std_msgs/Float32

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'std_msgs/Float32';
[data.data, info.data] = ros.internal.ros2.messages.ros2.default_type('single',1,0);
info.MessageType = 'std_msgs/Float32';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,1);
info.MatPath{1} = 'data';
