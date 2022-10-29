function [data, info] = timeReference
%TimeReference gives an empty data for sensor_msgs/TimeReference

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'sensor_msgs/TimeReference';
[data.header, info.header] = ros.internal.ros2.messages.std_msgs.header;
info.header.MLdataType = 'struct';
[data.time_ref, info.time_ref] = ros.internal.ros2.messages.builtin_interfaces.time;
info.time_ref.MLdataType = 'struct';
[data.source, info.source] = ros.internal.ros2.messages.ros2.char('string',1,NaN,0);
info.MessageType = 'sensor_msgs/TimeReference';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,9);
info.MatPath{1} = 'header';
info.MatPath{2} = 'header.stamp';
info.MatPath{3} = 'header.stamp.sec';
info.MatPath{4} = 'header.stamp.nanosec';
info.MatPath{5} = 'header.frame_id';
info.MatPath{6} = 'time_ref';
info.MatPath{7} = 'time_ref.sec';
info.MatPath{8} = 'time_ref.nanosec';
info.MatPath{9} = 'source';
