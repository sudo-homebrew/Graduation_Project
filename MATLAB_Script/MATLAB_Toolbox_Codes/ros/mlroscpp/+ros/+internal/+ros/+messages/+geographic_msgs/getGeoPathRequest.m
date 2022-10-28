function [data, info] = getGeoPathRequest
%GetGeoPath gives an empty data for geographic_msgs/GetGeoPathRequest

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'geographic_msgs/GetGeoPathRequest';
[data.Start, info.Start] = ros.internal.ros.messages.geographic_msgs.geoPoint;
info.Start.MLdataType = 'struct';
[data.Goal, info.Goal] = ros.internal.ros.messages.geographic_msgs.geoPoint;
info.Goal.MLdataType = 'struct';
info.MessageType = 'geographic_msgs/GetGeoPathRequest';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,8);
info.MatPath{1} = 'start';
info.MatPath{2} = 'start.latitude';
info.MatPath{3} = 'start.longitude';
info.MatPath{4} = 'start.altitude';
info.MatPath{5} = 'goal';
info.MatPath{6} = 'goal.latitude';
info.MatPath{7} = 'goal.longitude';
info.MatPath{8} = 'goal.altitude';
