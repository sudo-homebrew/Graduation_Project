function [data, info] = laserScan
%LaserScan gives an empty data for sensor_msgs/LaserScan

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'sensor_msgs/LaserScan';
[data.header, info.header] = ros.internal.ros2.messages.std_msgs.header;
info.header.MLdataType = 'struct';
[data.angle_min, info.angle_min] = ros.internal.ros2.messages.ros2.default_type('single',1,0);
[data.angle_max, info.angle_max] = ros.internal.ros2.messages.ros2.default_type('single',1,0);
[data.angle_increment, info.angle_increment] = ros.internal.ros2.messages.ros2.default_type('single',1,0);
[data.time_increment, info.time_increment] = ros.internal.ros2.messages.ros2.default_type('single',1,0);
[data.scan_time, info.scan_time] = ros.internal.ros2.messages.ros2.default_type('single',1,0);
[data.range_min, info.range_min] = ros.internal.ros2.messages.ros2.default_type('single',1,0);
[data.range_max, info.range_max] = ros.internal.ros2.messages.ros2.default_type('single',1,0);
[data.ranges, info.ranges] = ros.internal.ros2.messages.ros2.default_type('single',NaN,0);
[data.intensities, info.intensities] = ros.internal.ros2.messages.ros2.default_type('single',NaN,0);
info.MessageType = 'sensor_msgs/LaserScan';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,14);
info.MatPath{1} = 'header';
info.MatPath{2} = 'header.stamp';
info.MatPath{3} = 'header.stamp.sec';
info.MatPath{4} = 'header.stamp.nanosec';
info.MatPath{5} = 'header.frame_id';
info.MatPath{6} = 'angle_min';
info.MatPath{7} = 'angle_max';
info.MatPath{8} = 'angle_increment';
info.MatPath{9} = 'time_increment';
info.MatPath{10} = 'scan_time';
info.MatPath{11} = 'range_min';
info.MatPath{12} = 'range_max';
info.MatPath{13} = 'ranges';
info.MatPath{14} = 'intensities';
