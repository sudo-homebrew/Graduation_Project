function [data, info] = eSCInfoItem
%ESCInfoItem gives an empty data for mavros_msgs/ESCInfoItem

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'mavros_msgs/ESCInfoItem';
[data.Header, info.Header] = ros.internal.ros.messages.std_msgs.header;
info.Header.MLdataType = 'struct';
[data.FailureFlags, info.FailureFlags] = ros.internal.ros.messages.ros.default_type('uint16',1);
[data.ErrorCount, info.ErrorCount] = ros.internal.ros.messages.ros.default_type('uint32',1);
[data.Temperature, info.Temperature] = ros.internal.ros.messages.ros.default_type('uint8',1);
info.MessageType = 'mavros_msgs/ESCInfoItem';
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
info.MatPath{7} = 'failure_flags';
info.MatPath{8} = 'error_count';
info.MatPath{9} = 'temperature';
