function [data, info] = diagnosticArray
%DiagnosticArray gives an empty data for diagnostic_msgs/DiagnosticArray

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'diagnostic_msgs/DiagnosticArray';
[data.Header, info.Header] = ros.internal.ros.messages.std_msgs.header;
info.Header.MLdataType = 'struct';
[data.Status, info.Status] = ros.internal.ros.messages.diagnostic_msgs.diagnosticStatus;
info.Status.MLdataType = 'struct';
info.Status.MaxLen = NaN;
info.Status.MinLen = 0;
data.Status = data.Status([],1);
info.MessageType = 'diagnostic_msgs/DiagnosticArray';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,18);
info.MatPath{1} = 'header';
info.MatPath{2} = 'header.seq';
info.MatPath{3} = 'header.stamp';
info.MatPath{4} = 'header.stamp.sec';
info.MatPath{5} = 'header.stamp.nsec';
info.MatPath{6} = 'header.frame_id';
info.MatPath{7} = 'status';
info.MatPath{8} = 'status.OK';
info.MatPath{9} = 'status.WARN';
info.MatPath{10} = 'status.ERROR';
info.MatPath{11} = 'status.STALE';
info.MatPath{12} = 'status.level';
info.MatPath{13} = 'status.name';
info.MatPath{14} = 'status.message';
info.MatPath{15} = 'status.hardware_id';
info.MatPath{16} = 'status.values';
info.MatPath{17} = 'status.values.key';
info.MatPath{18} = 'status.values.value';
