function [data, info] = metricsMessage
%MetricsMessage gives an empty data for statistics_msgs/MetricsMessage

% Copyright 2019-2021 The MathWorks, Inc.
data = struct();
data.MessageType = 'statistics_msgs/MetricsMessage';
[data.measurement_source_name, info.measurement_source_name] = ros.internal.ros2.messages.ros2.char('string',1,NaN,0);
[data.metrics_source, info.metrics_source] = ros.internal.ros2.messages.ros2.char('string',1,NaN,0);
[data.unit, info.unit] = ros.internal.ros2.messages.ros2.char('string',1,NaN,0);
[data.window_start, info.window_start] = ros.internal.ros2.messages.builtin_interfaces.time;
info.window_start.MLdataType = 'struct';
[data.window_stop, info.window_stop] = ros.internal.ros2.messages.builtin_interfaces.time;
info.window_stop.MLdataType = 'struct';
[data.statistics, info.statistics] = ros.internal.ros2.messages.statistics_msgs.statisticDataPoint;
info.statistics.MLdataType = 'struct';
info.statistics.MaxLen = NaN;
info.statistics.MinLen = 0;
info.MessageType = 'statistics_msgs/MetricsMessage';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,12);
info.MatPath{1} = 'measurement_source_name';
info.MatPath{2} = 'metrics_source';
info.MatPath{3} = 'unit';
info.MatPath{4} = 'window_start';
info.MatPath{5} = 'window_start.sec';
info.MatPath{6} = 'window_start.nanosec';
info.MatPath{7} = 'window_stop';
info.MatPath{8} = 'window_stop.sec';
info.MatPath{9} = 'window_stop.nanosec';
info.MatPath{10} = 'statistics';
info.MatPath{11} = 'statistics.data_type';
info.MatPath{12} = 'statistics.data';
