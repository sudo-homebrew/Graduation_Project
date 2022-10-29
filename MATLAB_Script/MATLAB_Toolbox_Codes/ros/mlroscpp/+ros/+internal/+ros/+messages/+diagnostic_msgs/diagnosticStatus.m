function [data, info] = diagnosticStatus
%DiagnosticStatus gives an empty data for diagnostic_msgs/DiagnosticStatus

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'diagnostic_msgs/DiagnosticStatus';
[data.OK, info.OK] = ros.internal.ros.messages.ros.default_type('int8',1, 0);
[data.WARN, info.WARN] = ros.internal.ros.messages.ros.default_type('int8',1, 1);
[data.ERROR, info.ERROR] = ros.internal.ros.messages.ros.default_type('int8',1, 2);
[data.STALE, info.STALE] = ros.internal.ros.messages.ros.default_type('int8',1, 3);
[data.Level, info.Level] = ros.internal.ros.messages.ros.default_type('int8',1);
[data.Name, info.Name] = ros.internal.ros.messages.ros.char('string',0);
[data.Message, info.Message] = ros.internal.ros.messages.ros.char('string',0);
[data.HardwareId, info.HardwareId] = ros.internal.ros.messages.ros.char('string',0);
[data.Values, info.Values] = ros.internal.ros.messages.diagnostic_msgs.keyValue;
info.Values.MLdataType = 'struct';
info.Values.MaxLen = NaN;
info.Values.MinLen = 0;
data.Values = data.Values([],1);
info.MessageType = 'diagnostic_msgs/DiagnosticStatus';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,11);
info.MatPath{1} = 'OK';
info.MatPath{2} = 'WARN';
info.MatPath{3} = 'ERROR';
info.MatPath{4} = 'STALE';
info.MatPath{5} = 'level';
info.MatPath{6} = 'name';
info.MatPath{7} = 'message';
info.MatPath{8} = 'hardware_id';
info.MatPath{9} = 'values';
info.MatPath{10} = 'values.key';
info.MatPath{11} = 'values.value';
