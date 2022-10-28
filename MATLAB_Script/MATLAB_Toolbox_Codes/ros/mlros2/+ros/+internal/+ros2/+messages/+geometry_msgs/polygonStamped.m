function [data, info] = polygonStamped
%PolygonStamped gives an empty data for geometry_msgs/PolygonStamped

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'geometry_msgs/PolygonStamped';
[data.header, info.header] = ros.internal.ros2.messages.std_msgs.header;
info.header.MLdataType = 'struct';
[data.polygon, info.polygon] = ros.internal.ros2.messages.geometry_msgs.polygon;
info.polygon.MLdataType = 'struct';
info.MessageType = 'geometry_msgs/PolygonStamped';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,10);
info.MatPath{1} = 'header';
info.MatPath{2} = 'header.stamp';
info.MatPath{3} = 'header.stamp.sec';
info.MatPath{4} = 'header.stamp.nanosec';
info.MatPath{5} = 'header.frame_id';
info.MatPath{6} = 'polygon';
info.MatPath{7} = 'polygon.points';
info.MatPath{8} = 'polygon.points.x';
info.MatPath{9} = 'polygon.points.y';
info.MatPath{10} = 'polygon.points.z';
