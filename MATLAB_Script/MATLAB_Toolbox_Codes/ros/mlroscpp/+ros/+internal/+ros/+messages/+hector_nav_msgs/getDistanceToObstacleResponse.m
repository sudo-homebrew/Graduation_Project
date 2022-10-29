function [data, info] = getDistanceToObstacleResponse
%GetDistanceToObstacle gives an empty data for hector_nav_msgs/GetDistanceToObstacleResponse

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'hector_nav_msgs/GetDistanceToObstacleResponse';
[data.Distance, info.Distance] = ros.internal.ros.messages.ros.default_type('single',1);
[data.EndPoint, info.EndPoint] = ros.internal.ros.messages.geometry_msgs.pointStamped;
info.EndPoint.MLdataType = 'struct';
info.MessageType = 'hector_nav_msgs/GetDistanceToObstacleResponse';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,12);
info.MatPath{1} = 'distance';
info.MatPath{2} = 'end_point';
info.MatPath{3} = 'end_point.header';
info.MatPath{4} = 'end_point.header.seq';
info.MatPath{5} = 'end_point.header.stamp';
info.MatPath{6} = 'end_point.header.stamp.sec';
info.MatPath{7} = 'end_point.header.stamp.nsec';
info.MatPath{8} = 'end_point.header.frame_id';
info.MatPath{9} = 'end_point.point';
info.MatPath{10} = 'end_point.point.x';
info.MatPath{11} = 'end_point.point.y';
info.MatPath{12} = 'end_point.point.z';
