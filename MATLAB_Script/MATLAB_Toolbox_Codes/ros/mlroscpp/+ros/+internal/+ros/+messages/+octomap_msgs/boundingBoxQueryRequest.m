function [data, info] = boundingBoxQueryRequest
%BoundingBoxQuery gives an empty data for octomap_msgs/BoundingBoxQueryRequest

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'octomap_msgs/BoundingBoxQueryRequest';
[data.Min, info.Min] = ros.internal.ros.messages.geometry_msgs.point;
info.Min.MLdataType = 'struct';
[data.Max, info.Max] = ros.internal.ros.messages.geometry_msgs.point;
info.Max.MLdataType = 'struct';
info.MessageType = 'octomap_msgs/BoundingBoxQueryRequest';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,8);
info.MatPath{1} = 'min';
info.MatPath{2} = 'min.x';
info.MatPath{3} = 'min.y';
info.MatPath{4} = 'min.z';
info.MatPath{5} = 'max';
info.MatPath{6} = 'max.x';
info.MatPath{7} = 'max.y';
info.MatPath{8} = 'max.z';
