function [data, info] = logEntry
%LogEntry gives an empty data for mavros_msgs/LogEntry

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'mavros_msgs/LogEntry';
[data.Header, info.Header] = ros.internal.ros.messages.std_msgs.header;
info.Header.MLdataType = 'struct';
[data.Id, info.Id] = ros.internal.ros.messages.ros.default_type('uint16',1);
[data.NumLogs, info.NumLogs] = ros.internal.ros.messages.ros.default_type('uint16',1);
[data.LastLogNum, info.LastLogNum] = ros.internal.ros.messages.ros.default_type('uint16',1);
[data.TimeUtc, info.TimeUtc] = ros.internal.ros.messages.ros.time;
info.TimeUtc.MLdataType = 'struct';
[data.Size, info.Size] = ros.internal.ros.messages.ros.default_type('uint32',1);
info.MessageType = 'mavros_msgs/LogEntry';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,13);
info.MatPath{1} = 'header';
info.MatPath{2} = 'header.seq';
info.MatPath{3} = 'header.stamp';
info.MatPath{4} = 'header.stamp.sec';
info.MatPath{5} = 'header.stamp.nsec';
info.MatPath{6} = 'header.frame_id';
info.MatPath{7} = 'id';
info.MatPath{8} = 'num_logs';
info.MatPath{9} = 'last_log_num';
info.MatPath{10} = 'time_utc';
info.MatPath{11} = 'time_utc.sec';
info.MatPath{12} = 'time_utc.nsec';
info.MatPath{13} = 'size';
