function [data, info] = diagnosticStatus
%DiagnosticStatus gives an empty data for diagnostic_msgs/DiagnosticStatus

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'diagnostic_msgs/DiagnosticStatus';
[data.OK, info.OK] = ros.internal.ros2.messages.ros2.default_type('uint8',1,0, 0, [NaN]);
[data.WARN, info.WARN] = ros.internal.ros2.messages.ros2.default_type('uint8',1,0, 1, [NaN]);
[data.ERROR, info.ERROR] = ros.internal.ros2.messages.ros2.default_type('uint8',1,0, 2, [NaN]);
[data.STALE, info.STALE] = ros.internal.ros2.messages.ros2.default_type('uint8',1,0, 3, [NaN]);
[data.level, info.level] = ros.internal.ros2.messages.ros2.default_type('uint8',1,0);
[data.name, info.name] = ros.internal.ros2.messages.ros2.char('string',1,NaN,0);
[data.message, info.message] = ros.internal.ros2.messages.ros2.char('string',1,NaN,0);
[data.hardware_id, info.hardware_id] = ros.internal.ros2.messages.ros2.char('string',1,NaN,0);
[data.values, info.values] = ros.internal.ros2.messages.diagnostic_msgs.keyValue;
info.values.MLdataType = 'struct';
info.values.MaxLen = NaN;
info.values.MinLen = 0;
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
