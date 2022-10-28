function [data, info] = polygon
%Polygon gives an empty data for geometry_msgs/Polygon

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'geometry_msgs/Polygon';
[data.Points, info.Points] = ros.internal.ros.messages.geometry_msgs.point32;
info.Points.MLdataType = 'struct';
info.Points.MaxLen = NaN;
info.Points.MinLen = 0;
data.Points = data.Points([],1);
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
