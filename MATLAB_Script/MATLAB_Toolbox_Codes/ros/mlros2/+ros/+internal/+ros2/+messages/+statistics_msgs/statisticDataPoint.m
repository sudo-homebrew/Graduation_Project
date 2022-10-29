function [data, info] = statisticDataPoint
%StatisticDataPoint gives an empty data for statistics_msgs/StatisticDataPoint

% Copyright 2019-2021 The MathWorks, Inc.
data = struct();
data.MessageType = 'statistics_msgs/StatisticDataPoint';
[data.data_type, info.data_type] = ros.internal.ros2.messages.ros2.default_type('uint8',1,0);
[data.data, info.data] = ros.internal.ros2.messages.ros2.default_type('double',1,0);
info.MessageType = 'statistics_msgs/StatisticDataPoint';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,2);
info.MatPath{1} = 'data_type';
info.MatPath{2} = 'data';
