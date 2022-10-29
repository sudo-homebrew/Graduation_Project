function [data, info] = eSCStatusItem
%ESCStatusItem gives an empty data for mavros_msgs/ESCStatusItem

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'mavros_msgs/ESCStatusItem';
[data.Header, info.Header] = ros.internal.ros.messages.std_msgs.header;
info.Header.MLdataType = 'struct';
[data.Rpm, info.Rpm] = ros.internal.ros.messages.ros.default_type('int32',1);
[data.Voltage, info.Voltage] = ros.internal.ros.messages.ros.default_type('single',1);
[data.Current, info.Current] = ros.internal.ros.messages.ros.default_type('single',1);
info.MessageType = 'mavros_msgs/ESCStatusItem';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,9);
info.MatPath{1} = 'header';
info.MatPath{2} = 'header.seq';
info.MatPath{3} = 'header.stamp';
info.MatPath{4} = 'header.stamp.sec';
info.MatPath{5} = 'header.stamp.nsec';
info.MatPath{6} = 'header.frame_id';
info.MatPath{7} = 'rpm';
info.MatPath{8} = 'voltage';
info.MatPath{9} = 'current';
