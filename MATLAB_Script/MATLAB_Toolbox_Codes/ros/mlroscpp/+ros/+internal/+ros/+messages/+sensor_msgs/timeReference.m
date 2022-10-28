function [data, info] = timeReference
%TimeReference gives an empty data for sensor_msgs/TimeReference

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'sensor_msgs/TimeReference';
[data.Header, info.Header] = ros.internal.ros.messages.std_msgs.header;
info.Header.MLdataType = 'struct';
[data.TimeRef, info.TimeRef] = ros.internal.ros.messages.ros.time;
info.TimeRef.MLdataType = 'struct';
[data.Source, info.Source] = ros.internal.ros.messages.ros.char('string',0);
info.MessageType = 'sensor_msgs/TimeReference';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,10);
info.MatPath{1} = 'header';
info.MatPath{2} = 'header.seq';
info.MatPath{3} = 'header.stamp';
info.MatPath{4} = 'header.stamp.sec';
info.MatPath{5} = 'header.stamp.nsec';
info.MatPath{6} = 'header.frame_id';
info.MatPath{7} = 'time_ref';
info.MatPath{8} = 'time_ref.sec';
info.MatPath{9} = 'time_ref.nsec';
info.MatPath{10} = 'source';
