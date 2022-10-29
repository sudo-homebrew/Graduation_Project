function [data, info] = checkCircleRequest
%CheckCircle gives an empty data for jsk_pcl_ros/CheckCircleRequest

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'jsk_pcl_ros/CheckCircleRequest';
[data.Point, info.Point] = ros.internal.ros.messages.geometry_msgs.point;
info.Point.MLdataType = 'struct';
info.MessageType = 'jsk_pcl_ros/CheckCircleRequest';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,4);
info.MatPath{1} = 'point';
info.MatPath{2} = 'point.x';
info.MatPath{3} = 'point.y';
info.MatPath{4} = 'point.z';
