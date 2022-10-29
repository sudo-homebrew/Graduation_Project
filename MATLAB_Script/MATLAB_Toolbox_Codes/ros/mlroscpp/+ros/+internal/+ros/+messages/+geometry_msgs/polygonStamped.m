function [data, info] = polygonStamped
%PolygonStamped gives an empty data for geometry_msgs/PolygonStamped

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'geometry_msgs/PolygonStamped';
[data.Header, info.Header] = ros.internal.ros.messages.std_msgs.header;
info.Header.MLdataType = 'struct';
[data.Polygon, info.Polygon] = ros.internal.ros.messages.geometry_msgs.polygon;
info.Polygon.MLdataType = 'struct';
info.MessageType = 'geometry_msgs/PolygonStamped';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,11);
info.MatPath{1} = 'header';
info.MatPath{2} = 'header.seq';
info.MatPath{3} = 'header.stamp';
info.MatPath{4} = 'header.stamp.sec';
info.MatPath{5} = 'header.stamp.nsec';
info.MatPath{6} = 'header.frame_id';
info.MatPath{7} = 'polygon';
info.MatPath{8} = 'polygon.points';
info.MatPath{9} = 'polygon.points.x';
info.MatPath{10} = 'polygon.points.y';
info.MatPath{11} = 'polygon.points.z';
