function [data, info] = getPointMapROIRequest
%GetPointMapROI gives an empty data for map_msgs/GetPointMapROIRequest

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'map_msgs/GetPointMapROIRequest';
[data.X, info.X] = ros.internal.ros.messages.ros.default_type('double',1);
[data.Y, info.Y] = ros.internal.ros.messages.ros.default_type('double',1);
[data.Z, info.Z] = ros.internal.ros.messages.ros.default_type('double',1);
[data.R, info.R] = ros.internal.ros.messages.ros.default_type('double',1);
[data.LX, info.LX] = ros.internal.ros.messages.ros.default_type('double',1);
[data.LY, info.LY] = ros.internal.ros.messages.ros.default_type('double',1);
[data.LZ, info.LZ] = ros.internal.ros.messages.ros.default_type('double',1);
info.MessageType = 'map_msgs/GetPointMapROIRequest';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,7);
info.MatPath{1} = 'x';
info.MatPath{2} = 'y';
info.MatPath{3} = 'z';
info.MatPath{4} = 'r';
info.MatPath{5} = 'l_x';
info.MatPath{6} = 'l_y';
info.MatPath{7} = 'l_z';
