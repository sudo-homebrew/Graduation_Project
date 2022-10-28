function [data, info] = getMapROIRequest
%GetMapROI gives an empty data for map_msgs/GetMapROIRequest

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'map_msgs/GetMapROIRequest';
[data.X, info.X] = ros.internal.ros.messages.ros.default_type('double',1);
[data.Y, info.Y] = ros.internal.ros.messages.ros.default_type('double',1);
[data.LX, info.LX] = ros.internal.ros.messages.ros.default_type('double',1);
[data.LY, info.LY] = ros.internal.ros.messages.ros.default_type('double',1);
info.MessageType = 'map_msgs/GetMapROIRequest';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,4);
info.MatPath{1} = 'x';
info.MatPath{2} = 'y';
info.MatPath{3} = 'l_x';
info.MatPath{4} = 'l_y';
