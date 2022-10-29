function [data, info] = rawPPS
%RawPPS gives an empty data for applanix_msgs/RawPPS

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'applanix_msgs/RawPPS';
[data.Td, info.Td] = ros.internal.ros.messages.applanix_msgs.timeDistance;
info.Td.MLdataType = 'struct';
[data.PpsCount, info.PpsCount] = ros.internal.ros.messages.ros.default_type('uint32',1);
info.MessageType = 'applanix_msgs/RawPPS';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,7);
info.MatPath{1} = 'td';
info.MatPath{2} = 'td.time1';
info.MatPath{3} = 'td.time2';
info.MatPath{4} = 'td.distance';
info.MatPath{5} = 'td.time_types';
info.MatPath{6} = 'td.distance_type';
info.MatPath{7} = 'pps_count';
