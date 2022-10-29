function [data, info] = digital
%Digital gives an empty data for arbotix_msgs/Digital

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'arbotix_msgs/Digital';
[data.Header, info.Header] = ros.internal.ros.messages.std_msgs.header;
info.Header.MLdataType = 'struct';
[data.LOW, info.LOW] = ros.internal.ros.messages.ros.default_type('uint8',1, 0);
[data.HIGH, info.HIGH] = ros.internal.ros.messages.ros.default_type('uint8',1, 255);
[data.Value, info.Value] = ros.internal.ros.messages.ros.default_type('uint8',1);
[data.INPUT, info.INPUT] = ros.internal.ros.messages.ros.default_type('uint8',1, 0);
[data.OUTPUT, info.OUTPUT] = ros.internal.ros.messages.ros.default_type('uint8',1, 255);
[data.Direction, info.Direction] = ros.internal.ros.messages.ros.default_type('uint8',1);
info.MessageType = 'arbotix_msgs/Digital';
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
info.MatPath{7} = 'LOW';
info.MatPath{8} = 'HIGH';
info.MatPath{9} = 'value';
info.MatPath{10} = 'INPUT';
info.MatPath{11} = 'OUTPUT';
info.MatPath{12} = 'direction';
