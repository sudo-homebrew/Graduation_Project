function [data, info] = color
%Color gives an empty data for turtlesim/Color

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'turtlesim/Color';
[data.R, info.R] = ros.internal.ros.messages.ros.default_type('uint8',1);
[data.G, info.G] = ros.internal.ros.messages.ros.default_type('uint8',1);
[data.B, info.B] = ros.internal.ros.messages.ros.default_type('uint8',1);
info.MessageType = 'turtlesim/Color';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,3);
info.MatPath{1} = 'r';
info.MatPath{2} = 'g';
info.MatPath{3} = 'b';
