function [data, info] = rangeArray
%RangeArray gives an empty data for segbot_sensors/RangeArray

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'segbot_sensors/RangeArray';
[data.Ranges, info.Ranges] = ros.internal.ros.messages.sensor_msgs.range;
info.Ranges.MLdataType = 'struct';
info.Ranges.MaxLen = NaN;
info.Ranges.MinLen = 0;
data.Ranges = data.Ranges([],1);
info.MessageType = 'segbot_sensors/RangeArray';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,14);
info.MatPath{1} = 'ranges';
info.MatPath{2} = 'ranges.header';
info.MatPath{3} = 'ranges.header.seq';
info.MatPath{4} = 'ranges.header.stamp';
info.MatPath{5} = 'ranges.header.stamp.sec';
info.MatPath{6} = 'ranges.header.stamp.nsec';
info.MatPath{7} = 'ranges.header.frame_id';
info.MatPath{8} = 'ranges.ULTRASOUND';
info.MatPath{9} = 'ranges.INFRARED';
info.MatPath{10} = 'ranges.radiation_type';
info.MatPath{11} = 'ranges.field_of_view';
info.MatPath{12} = 'ranges.min_range';
info.MatPath{13} = 'ranges.max_range';
info.MatPath{14} = 'ranges.range';
