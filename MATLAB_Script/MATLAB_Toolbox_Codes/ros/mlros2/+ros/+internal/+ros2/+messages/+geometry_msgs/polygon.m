function [data, info] = polygon
%Polygon gives an empty data for geometry_msgs/Polygon

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'geometry_msgs/Polygon';
[data.points, info.points] = ros.internal.ros2.messages.geometry_msgs.point32;
info.points.MLdataType = 'struct';
info.points.MaxLen = NaN;
info.points.MinLen = 0;
info.MessageType = 'geometry_msgs/Polygon';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,4);
info.MatPath{1} = 'points';
info.MatPath{2} = 'points.x';
info.MatPath{3} = 'points.y';
info.MatPath{4} = 'points.z';
