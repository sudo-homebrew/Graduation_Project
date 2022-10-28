function [data, info] = callPolygonResponse
%CallPolygon gives an empty data for jsk_pcl_ros/CallPolygonResponse

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'jsk_pcl_ros/CallPolygonResponse';
[data.Points, info.Points] = ros.internal.ros.messages.geometry_msgs.polygonStamped;
info.Points.MLdataType = 'struct';
info.MessageType = 'jsk_pcl_ros/CallPolygonResponse';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,12);
info.MatPath{1} = 'points';
info.MatPath{2} = 'points.header';
info.MatPath{3} = 'points.header.seq';
info.MatPath{4} = 'points.header.stamp';
info.MatPath{5} = 'points.header.stamp.sec';
info.MatPath{6} = 'points.header.stamp.nsec';
info.MatPath{7} = 'points.header.frame_id';
info.MatPath{8} = 'points.polygon';
info.MatPath{9} = 'points.polygon.points';
info.MatPath{10} = 'points.polygon.points.x';
info.MatPath{11} = 'points.polygon.points.y';
info.MatPath{12} = 'points.polygon.points.z';
