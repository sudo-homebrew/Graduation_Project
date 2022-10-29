function [data, info] = rawDMI
%RawDMI gives an empty data for applanix_msgs/RawDMI

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'applanix_msgs/RawDMI';
[data.Td, info.Td] = ros.internal.ros.messages.applanix_msgs.timeDistance;
info.Td.MLdataType = 'struct';
[data.UpDownPulseCount, info.UpDownPulseCount] = ros.internal.ros.messages.ros.default_type('int32',1);
[data.RectifiedPulseCount, info.RectifiedPulseCount] = ros.internal.ros.messages.ros.default_type('uint32',1);
[data.EventCount, info.EventCount] = ros.internal.ros.messages.ros.default_type('int32',1);
[data.ReservedCount, info.ReservedCount] = ros.internal.ros.messages.ros.default_type('uint32',1);
info.MessageType = 'applanix_msgs/RawDMI';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,10);
info.MatPath{1} = 'td';
info.MatPath{2} = 'td.time1';
info.MatPath{3} = 'td.time2';
info.MatPath{4} = 'td.distance';
info.MatPath{5} = 'td.time_types';
info.MatPath{6} = 'td.distance_type';
info.MatPath{7} = 'up_down_pulse_count';
info.MatPath{8} = 'rectified_pulse_count';
info.MatPath{9} = 'event_count';
info.MatPath{10} = 'reserved_count';
