function [data, info] = logData
%LogData gives an empty data for mavros_msgs/LogData

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'mavros_msgs/LogData';
[data.Header, info.Header] = ros.internal.ros.messages.std_msgs.header;
info.Header.MLdataType = 'struct';
[data.Id, info.Id] = ros.internal.ros.messages.ros.default_type('uint16',1);
[data.Offset, info.Offset] = ros.internal.ros.messages.ros.default_type('uint32',1);
[data.Data, info.Data] = ros.internal.ros.messages.ros.default_type('uint8',NaN);
info.MessageType = 'mavros_msgs/LogData';
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
info.MatPath{7} = 'id';
info.MatPath{8} = 'offset';
info.MatPath{9} = 'data';
