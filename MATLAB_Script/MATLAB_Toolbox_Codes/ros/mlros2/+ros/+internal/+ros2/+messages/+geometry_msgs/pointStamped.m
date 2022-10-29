function [data, info] = pointStamped
%PointStamped gives an empty data for geometry_msgs/PointStamped

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'geometry_msgs/PointStamped';
[data.header, info.header] = ros.internal.ros2.messages.std_msgs.header;
info.header.MLdataType = 'struct';
[data.point, info.point] = ros.internal.ros2.messages.geometry_msgs.point;
info.point.MLdataType = 'struct';
info.MessageType = 'geometry_msgs/PointStamped';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,9);
info.MatPath{1} = 'header';
info.MatPath{2} = 'header.stamp';
info.MatPath{3} = 'header.stamp.sec';
info.MatPath{4} = 'header.stamp.nanosec';
info.MatPath{5} = 'header.frame_id';
info.MatPath{6} = 'point';
info.MatPath{7} = 'point.x';
info.MatPath{8} = 'point.y';
info.MatPath{9} = 'point.z';
