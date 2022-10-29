function [data, info] = polygonArray
%PolygonArray gives an empty data for jsk_pcl_ros/PolygonArray

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'jsk_pcl_ros/PolygonArray';
[data.Header, info.Header] = ros.internal.ros.messages.std_msgs.header;
info.Header.MLdataType = 'struct';
[data.Polygons, info.Polygons] = ros.internal.ros.messages.geometry_msgs.polygonStamped;
info.Polygons.MLdataType = 'struct';
info.Polygons.MaxLen = NaN;
info.Polygons.MinLen = 0;
data.Polygons = data.Polygons([],1);
info.MessageType = 'jsk_pcl_ros/PolygonArray';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,18);
info.MatPath{1} = 'header';
info.MatPath{2} = 'header.seq';
info.MatPath{3} = 'header.stamp';
info.MatPath{4} = 'header.stamp.sec';
info.MatPath{5} = 'header.stamp.nsec';
info.MatPath{6} = 'header.frame_id';
info.MatPath{7} = 'polygons';
info.MatPath{8} = 'polygons.header';
info.MatPath{9} = 'polygons.header.seq';
info.MatPath{10} = 'polygons.header.stamp';
info.MatPath{11} = 'polygons.header.stamp.sec';
info.MatPath{12} = 'polygons.header.stamp.nsec';
info.MatPath{13} = 'polygons.header.frame_id';
info.MatPath{14} = 'polygons.polygon';
info.MatPath{15} = 'polygons.polygon.points';
info.MatPath{16} = 'polygons.polygon.points.x';
info.MatPath{17} = 'polygons.polygon.points.y';
info.MatPath{18} = 'polygons.polygon.points.z';
