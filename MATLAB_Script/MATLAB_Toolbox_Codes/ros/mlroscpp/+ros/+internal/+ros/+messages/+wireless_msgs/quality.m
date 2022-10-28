function [data, info] = quality
%Quality gives an empty data for wireless_msgs/Quality

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'wireless_msgs/Quality';
[data.Header, info.Header] = ros.internal.ros.messages.std_msgs.header;
info.Header.MLdataType = 'struct';
[data.MessagesReceived, info.MessagesReceived] = ros.internal.ros.messages.ros.default_type('uint16',1);
[data.MessagesMissed, info.MessagesMissed] = ros.internal.ros.messages.ros.default_type('uint16',1);
[data.TotalLength, info.TotalLength] = ros.internal.ros.messages.ros.default_type('uint32',1);
[data.MessageLengths, info.MessageLengths] = ros.internal.ros.messages.ros.default_type('uint32',NaN);
[data.LatencyAvg, info.LatencyAvg] = ros.internal.ros.messages.ros.default_type('single',1);
[data.LatencyMeasurements, info.LatencyMeasurements] = ros.internal.ros.messages.ros.default_type('single',NaN);
info.MessageType = 'wireless_msgs/Quality';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,12);
info.MatPath{1} = 'header';
info.MatPath{2} = 'header.seq';
info.MatPath{3} = 'header.stamp';
info.MatPath{4} = 'header.stamp.sec';
info.MatPath{5} = 'header.stamp.nsec';
info.MatPath{6} = 'header.frame_id';
info.MatPath{7} = 'messages_received';
info.MatPath{8} = 'messages_missed';
info.MatPath{9} = 'total_length';
info.MatPath{10} = 'message_lengths';
info.MatPath{11} = 'latency_avg';
info.MatPath{12} = 'latency_measurements';
