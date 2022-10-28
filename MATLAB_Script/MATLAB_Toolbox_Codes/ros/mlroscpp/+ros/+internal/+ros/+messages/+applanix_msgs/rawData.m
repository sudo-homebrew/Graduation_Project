function [data, info] = rawData
%RawData gives an empty data for applanix_msgs/RawData

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'applanix_msgs/RawData';
[data.Td, info.Td] = ros.internal.ros.messages.applanix_msgs.timeDistance;
info.Td.MLdataType = 'struct';
[data.ReceiverType, info.ReceiverType] = ros.internal.ros.messages.ros.default_type('uint16',1);
[data.Reserved, info.Reserved] = ros.internal.ros.messages.ros.default_type('uint32',1);
[data.Message, info.Message] = ros.internal.ros.messages.ros.default_type('uint8',NaN);
info.MessageType = 'applanix_msgs/RawData';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,9);
info.MatPath{1} = 'td';
info.MatPath{2} = 'td.time1';
info.MatPath{3} = 'td.time2';
info.MatPath{4} = 'td.distance';
info.MatPath{5} = 'td.time_types';
info.MatPath{6} = 'td.distance_type';
info.MatPath{7} = 'receiver_type';
info.MatPath{8} = 'reserved';
info.MatPath{9} = 'message';
