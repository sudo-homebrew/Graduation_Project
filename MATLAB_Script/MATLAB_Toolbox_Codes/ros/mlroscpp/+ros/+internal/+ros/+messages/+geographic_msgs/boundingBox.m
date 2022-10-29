function [data, info] = boundingBox
%BoundingBox gives an empty data for geographic_msgs/BoundingBox

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'geographic_msgs/BoundingBox';
[data.MinPt, info.MinPt] = ros.internal.ros.messages.geographic_msgs.geoPoint;
info.MinPt.MLdataType = 'struct';
[data.MaxPt, info.MaxPt] = ros.internal.ros.messages.geographic_msgs.geoPoint;
info.MaxPt.MLdataType = 'struct';
info.MessageType = 'geographic_msgs/BoundingBox';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,8);
info.MatPath{1} = 'min_pt';
info.MatPath{2} = 'min_pt.latitude';
info.MatPath{3} = 'min_pt.longitude';
info.MatPath{4} = 'min_pt.altitude';
info.MatPath{5} = 'max_pt';
info.MatPath{6} = 'max_pt.latitude';
info.MatPath{7} = 'max_pt.longitude';
info.MatPath{8} = 'max_pt.altitude';
