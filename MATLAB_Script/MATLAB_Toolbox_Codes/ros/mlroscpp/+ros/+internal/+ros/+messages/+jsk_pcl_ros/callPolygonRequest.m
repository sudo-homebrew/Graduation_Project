function [data, info] = callPolygonRequest
%CallPolygon gives an empty data for jsk_pcl_ros/CallPolygonRequest

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'jsk_pcl_ros/CallPolygonRequest';
[data.Filename, info.Filename] = ros.internal.ros.messages.ros.char('string',0);
info.MessageType = 'jsk_pcl_ros/CallPolygonRequest';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,1);
info.MatPath{1} = 'filename';
