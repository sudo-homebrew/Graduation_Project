function [data, info] = getDistanceToObstacleRequest
%GetDistanceToObstacle gives an empty data for hector_nav_msgs/GetDistanceToObstacleRequest

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'hector_nav_msgs/GetDistanceToObstacleRequest';
[data.Point, info.Point] = ros.internal.ros.messages.geometry_msgs.pointStamped;
info.Point.MLdataType = 'struct';
info.MessageType = 'hector_nav_msgs/GetDistanceToObstacleRequest';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,11);
info.MatPath{1} = 'point';
info.MatPath{2} = 'point.header';
info.MatPath{3} = 'point.header.seq';
info.MatPath{4} = 'point.header.stamp';
info.MatPath{5} = 'point.header.stamp.sec';
info.MatPath{6} = 'point.header.stamp.nsec';
info.MatPath{7} = 'point.header.frame_id';
info.MatPath{8} = 'point.point';
info.MatPath{9} = 'point.point.x';
info.MatPath{10} = 'point.point.y';
info.MatPath{11} = 'point.point.z';
