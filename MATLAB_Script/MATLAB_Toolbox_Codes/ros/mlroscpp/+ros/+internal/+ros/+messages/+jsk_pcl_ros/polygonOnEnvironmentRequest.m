function [data, info] = polygonOnEnvironmentRequest
%PolygonOnEnvironment gives an empty data for jsk_pcl_ros/PolygonOnEnvironmentRequest

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'jsk_pcl_ros/PolygonOnEnvironmentRequest';
[data.EnvironmentId, info.EnvironmentId] = ros.internal.ros.messages.ros.default_type('uint32',1);
[data.PlaneIndex, info.PlaneIndex] = ros.internal.ros.messages.ros.default_type('uint32',1);
[data.Polygon, info.Polygon] = ros.internal.ros.messages.geometry_msgs.polygonStamped;
info.Polygon.MLdataType = 'struct';
info.MessageType = 'jsk_pcl_ros/PolygonOnEnvironmentRequest';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,14);
info.MatPath{1} = 'environment_id';
info.MatPath{2} = 'plane_index';
info.MatPath{3} = 'polygon';
info.MatPath{4} = 'polygon.header';
info.MatPath{5} = 'polygon.header.seq';
info.MatPath{6} = 'polygon.header.stamp';
info.MatPath{7} = 'polygon.header.stamp.sec';
info.MatPath{8} = 'polygon.header.stamp.nsec';
info.MatPath{9} = 'polygon.header.frame_id';
info.MatPath{10} = 'polygon.polygon';
info.MatPath{11} = 'polygon.polygon.points';
info.MatPath{12} = 'polygon.polygon.points.x';
info.MatPath{13} = 'polygon.polygon.points.y';
info.MatPath{14} = 'polygon.polygon.points.z';
