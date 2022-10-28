function [data, info] = colorRGBA
%ColorRGBA gives an empty data for std_msgs/ColorRGBA

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'std_msgs/ColorRGBA';
[data.R, info.R] = ros.internal.ros.messages.ros.default_type('single',1);
[data.G, info.G] = ros.internal.ros.messages.ros.default_type('single',1);
[data.B, info.B] = ros.internal.ros.messages.ros.default_type('single',1);
[data.A, info.A] = ros.internal.ros.messages.ros.default_type('single',1);
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
