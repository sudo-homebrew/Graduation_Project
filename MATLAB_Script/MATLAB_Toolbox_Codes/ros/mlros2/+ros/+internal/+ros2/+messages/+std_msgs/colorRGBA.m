function [data, info] = colorRGBA
%ColorRGBA gives an empty data for std_msgs/ColorRGBA

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'std_msgs/ColorRGBA';
[data.r, info.r] = ros.internal.ros2.messages.ros2.default_type('single',1,0);
[data.g, info.g] = ros.internal.ros2.messages.ros2.default_type('single',1,0);
[data.b, info.b] = ros.internal.ros2.messages.ros2.default_type('single',1,0);
[data.a, info.a] = ros.internal.ros2.messages.ros2.default_type('single',1,0);
info.MessageType = 'std_msgs/ColorRGBA';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,4);
info.MatPath{1} = 'r';
info.MatPath{2} = 'g';
info.MatPath{3} = 'b';
info.MatPath{4} = 'a';
