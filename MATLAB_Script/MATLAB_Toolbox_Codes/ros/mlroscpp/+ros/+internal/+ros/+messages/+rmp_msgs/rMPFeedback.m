function [data, info] = rMPFeedback
%RMPFeedback gives an empty data for rmp_msgs/RMPFeedback

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'rmp_msgs/RMPFeedback';
[data.Header, info.Header] = ros.internal.ros.messages.std_msgs.header;
info.Header.MLdataType = 'struct';
[data.SensorItems, info.SensorItems] = ros.internal.ros.messages.ros.char('string',NaN);
[data.SensorValues, info.SensorValues] = ros.internal.ros.messages.ros.default_type('single',NaN);
[data.FaultStatusItems, info.FaultStatusItems] = ros.internal.ros.messages.ros.char('string',NaN);
[data.FaultStatusValues, info.FaultStatusValues] = ros.internal.ros.messages.ros.default_type('uint32',NaN);
[data.IpInfo, info.IpInfo] = ros.internal.ros.messages.ros.char('string',NaN);
[data.IpValues, info.IpValues] = ros.internal.ros.messages.ros.char('string',NaN);
info.MessageType = 'rmp_msgs/RMPFeedback';
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
info.MatPath{7} = 'sensor_items';
info.MatPath{8} = 'sensor_values';
info.MatPath{9} = 'fault_status_items';
info.MatPath{10} = 'fault_status_values';
info.MatPath{11} = 'ip_info';
info.MatPath{12} = 'ip_values';
