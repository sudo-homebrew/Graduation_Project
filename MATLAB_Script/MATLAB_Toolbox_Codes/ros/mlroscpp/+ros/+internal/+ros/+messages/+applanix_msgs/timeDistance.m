function [data, info] = timeDistance
%TimeDistance gives an empty data for applanix_msgs/TimeDistance

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'applanix_msgs/TimeDistance';
[data.Time1, info.Time1] = ros.internal.ros.messages.ros.default_type('double',1);
[data.Time2, info.Time2] = ros.internal.ros.messages.ros.default_type('double',1);
[data.Distance, info.Distance] = ros.internal.ros.messages.ros.default_type('double',1);
[data.TimeTypes, info.TimeTypes] = ros.internal.ros.messages.ros.default_type('uint8',1);
[data.DistanceType, info.DistanceType] = ros.internal.ros.messages.ros.default_type('uint8',1);
info.MessageType = 'applanix_msgs/TimeDistance';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,5);
info.MatPath{1} = 'time1';
info.MatPath{2} = 'time2';
info.MatPath{3} = 'distance';
info.MatPath{4} = 'time_types';
info.MatPath{5} = 'distance_type';
