function [data, info] = getMapROIRequest
%GetMapROI gives an empty data for map_msgs/GetMapROIRequest

% Copyright 2019-2021 The MathWorks, Inc.
data = struct();
data.MessageType = 'map_msgs/GetMapROIRequest';
[data.x, info.x] = ros.internal.ros2.messages.ros2.default_type('double',1,0);
[data.y, info.y] = ros.internal.ros2.messages.ros2.default_type('double',1,0);
[data.l_x, info.l_x] = ros.internal.ros2.messages.ros2.default_type('double',1,0);
[data.l_y, info.l_y] = ros.internal.ros2.messages.ros2.default_type('double',1,0);
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
