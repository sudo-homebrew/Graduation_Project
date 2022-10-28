function [data, info] = pointStamped
%PointStamped gives an empty data for geometry_msgs/PointStamped

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'geometry_msgs/PointStamped';
[data.Header, info.Header] = ros.internal.ros.messages.std_msgs.header;
info.Header.MLdataType = 'struct';
[data.Point, info.Point] = ros.internal.ros.messages.geometry_msgs.point;
info.Point.MLdataType = 'struct';
info.MessageType = 'geometry_msgs/PointStamped';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,10);
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
