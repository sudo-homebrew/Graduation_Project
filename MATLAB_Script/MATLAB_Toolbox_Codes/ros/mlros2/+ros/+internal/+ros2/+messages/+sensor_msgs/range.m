function [data, info] = range
%Range gives an empty data for sensor_msgs/Range

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'sensor_msgs/Range';
[data.header, info.header] = ros.internal.ros2.messages.std_msgs.header;
info.header.MLdataType = 'struct';
[data.ULTRASOUND, info.ULTRASOUND] = ros.internal.ros2.messages.ros2.default_type('uint8',1,0, 0, [NaN]);
[data.INFRARED, info.INFRARED] = ros.internal.ros2.messages.ros2.default_type('uint8',1,0, 1, [NaN]);
[data.radiation_type, info.radiation_type] = ros.internal.ros2.messages.ros2.default_type('uint8',1,0);
[data.field_of_view, info.field_of_view] = ros.internal.ros2.messages.ros2.default_type('single',1,0);
[data.min_range, info.min_range] = ros.internal.ros2.messages.ros2.default_type('single',1,0);
[data.max_range, info.max_range] = ros.internal.ros2.messages.ros2.default_type('single',1,0);
[data.range, info.range] = ros.internal.ros2.messages.ros2.default_type('single',1,0);
info.MessageType = 'sensor_msgs/Range';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,12);
info.MatPath{1} = 'header';
info.MatPath{2} = 'header.stamp';
info.MatPath{3} = 'header.stamp.sec';
info.MatPath{4} = 'header.stamp.nanosec';
info.MatPath{5} = 'header.frame_id';
info.MatPath{6} = 'ULTRASOUND';
info.MatPath{7} = 'INFRARED';
info.MatPath{8} = 'radiation_type';
info.MatPath{9} = 'field_of_view';
info.MatPath{10} = 'min_range';
info.MatPath{11} = 'max_range';
info.MatPath{12} = 'range';
