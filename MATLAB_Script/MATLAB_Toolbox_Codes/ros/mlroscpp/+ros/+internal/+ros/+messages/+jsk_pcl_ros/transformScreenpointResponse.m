function [data, info] = transformScreenpointResponse
%TransformScreenpoint gives an empty data for jsk_pcl_ros/TransformScreenpointResponse

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'jsk_pcl_ros/TransformScreenpointResponse';
[data.Header, info.Header] = ros.internal.ros.messages.std_msgs.header;
info.Header.MLdataType = 'struct';
[data.Point, info.Point] = ros.internal.ros.messages.geometry_msgs.point;
info.Point.MLdataType = 'struct';
[data.Vector, info.Vector] = ros.internal.ros.messages.geometry_msgs.vector3;
info.Vector.MLdataType = 'struct';
info.MessageType = 'jsk_pcl_ros/TransformScreenpointResponse';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,14);
info.MatPath{1} = 'header';
info.MatPath{2} = 'header.seq';
info.MatPath{3} = 'header.stamp';
info.MatPath{4} = 'header.stamp.sec';
info.MatPath{5} = 'header.stamp.nsec';
info.MatPath{6} = 'header.frame_id';
info.MatPath{7} = 'point';
info.MatPath{8} = 'point.x';
info.MatPath{9} = 'point.y';
info.MatPath{10} = 'point.z';
info.MatPath{11} = 'vector';
info.MatPath{12} = 'vector.x';
info.MatPath{13} = 'vector.y';
info.MatPath{14} = 'vector.z';
