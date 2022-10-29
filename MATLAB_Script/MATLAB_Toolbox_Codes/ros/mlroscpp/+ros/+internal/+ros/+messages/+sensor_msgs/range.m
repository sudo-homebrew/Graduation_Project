function [data, info] = range
%Range gives an empty data for sensor_msgs/Range

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'sensor_msgs/Range';
[data.Header, info.Header] = ros.internal.ros.messages.std_msgs.header;
info.Header.MLdataType = 'struct';
[data.ULTRASOUND, info.ULTRASOUND] = ros.internal.ros.messages.ros.default_type('uint8',1, 0);
[data.INFRARED, info.INFRARED] = ros.internal.ros.messages.ros.default_type('uint8',1, 1);
[data.RadiationType, info.RadiationType] = ros.internal.ros.messages.ros.default_type('uint8',1);
[data.FieldOfView, info.FieldOfView] = ros.internal.ros.messages.ros.default_type('single',1);
[data.MinRange, info.MinRange] = ros.internal.ros.messages.ros.default_type('single',1);
[data.MaxRange, info.MaxRange] = ros.internal.ros.messages.ros.default_type('single',1);
[data.Range_, info.Range_] = ros.internal.ros.messages.ros.default_type('single',1);
info.MessageType = 'sensor_msgs/Range';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,13);
info.MatPath{1} = 'header';
info.MatPath{2} = 'header.seq';
info.MatPath{3} = 'header.stamp';
info.MatPath{4} = 'header.stamp.sec';
info.MatPath{5} = 'header.stamp.nsec';
info.MatPath{6} = 'header.frame_id';
info.MatPath{7} = 'ULTRASOUND';
info.MatPath{8} = 'INFRARED';
info.MatPath{9} = 'radiation_type';
info.MatPath{10} = 'field_of_view';
info.MatPath{11} = 'min_range';
info.MatPath{12} = 'max_range';
info.MatPath{13} = 'range';
