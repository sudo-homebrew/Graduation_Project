function [data, info] = statisticDataType
%StatisticDataType gives an empty data for statistics_msgs/StatisticDataType

% Copyright 2019-2021 The MathWorks, Inc.
data = struct();
data.MessageType = 'statistics_msgs/StatisticDataType';
[data.STATISTICS_DATA_TYPE_UNINITIALIZED, info.STATISTICS_DATA_TYPE_UNINITIALIZED] = ros.internal.ros2.messages.ros2.default_type('uint8',1,0, 0, [NaN]);
[data.STATISTICS_DATA_TYPE_AVERAGE, info.STATISTICS_DATA_TYPE_AVERAGE] = ros.internal.ros2.messages.ros2.default_type('uint8',1,0, 1, [NaN]);
[data.STATISTICS_DATA_TYPE_MINIMUM, info.STATISTICS_DATA_TYPE_MINIMUM] = ros.internal.ros2.messages.ros2.default_type('uint8',1,0, 2, [NaN]);
[data.STATISTICS_DATA_TYPE_MAXIMUM, info.STATISTICS_DATA_TYPE_MAXIMUM] = ros.internal.ros2.messages.ros2.default_type('uint8',1,0, 3, [NaN]);
[data.STATISTICS_DATA_TYPE_STDDEV, info.STATISTICS_DATA_TYPE_STDDEV] = ros.internal.ros2.messages.ros2.default_type('uint8',1,0, 4, [NaN]);
[data.STATISTICS_DATA_TYPE_SAMPLE_COUNT, info.STATISTICS_DATA_TYPE_SAMPLE_COUNT] = ros.internal.ros2.messages.ros2.default_type('uint8',1,0, 5, [NaN]);
info.MessageType = 'statistics_msgs/StatisticDataType';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,6);
info.MatPath{1} = 'STATISTICS_DATA_TYPE_UNINITIALIZED';
info.MatPath{2} = 'STATISTICS_DATA_TYPE_AVERAGE';
info.MatPath{3} = 'STATISTICS_DATA_TYPE_MINIMUM';
info.MatPath{4} = 'STATISTICS_DATA_TYPE_MAXIMUM';
info.MatPath{5} = 'STATISTICS_DATA_TYPE_STDDEV';
info.MatPath{6} = 'STATISTICS_DATA_TYPE_SAMPLE_COUNT';
