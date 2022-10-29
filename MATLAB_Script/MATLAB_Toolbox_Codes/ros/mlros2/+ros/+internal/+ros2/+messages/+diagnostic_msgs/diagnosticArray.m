function [data, info] = diagnosticArray
%DiagnosticArray gives an empty data for diagnostic_msgs/DiagnosticArray

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'diagnostic_msgs/DiagnosticArray';
[data.header, info.header] = ros.internal.ros2.messages.std_msgs.header;
info.header.MLdataType = 'struct';
[data.status, info.status] = ros.internal.ros2.messages.diagnostic_msgs.diagnosticStatus;
info.status.MLdataType = 'struct';
info.status.MaxLen = NaN;
info.status.MinLen = 0;
info.MessageType = 'diagnostic_msgs/DiagnosticArray';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,17);
info.MatPath{1} = 'header';
info.MatPath{2} = 'header.stamp';
info.MatPath{3} = 'header.stamp.sec';
info.MatPath{4} = 'header.stamp.nanosec';
info.MatPath{5} = 'header.frame_id';
info.MatPath{6} = 'status';
info.MatPath{7} = 'status.OK';
info.MatPath{8} = 'status.WARN';
info.MatPath{9} = 'status.ERROR';
info.MatPath{10} = 'status.STALE';
info.MatPath{11} = 'status.level';
info.MatPath{12} = 'status.name';
info.MatPath{13} = 'status.message';
info.MatPath{14} = 'status.hardware_id';
info.MatPath{15} = 'status.values';
info.MatPath{16} = 'status.values.key';
info.MatPath{17} = 'status.values.value';
