function [data, info] = selfTestResponse
%SelfTest gives an empty data for diagnostic_msgs/SelfTestResponse

% Copyright 2019-2021 The MathWorks, Inc.
data = struct();
data.MessageType = 'diagnostic_msgs/SelfTestResponse';
[data.id, info.id] = ros.internal.ros2.messages.ros2.char('string',1,NaN,0);
[data.passed, info.passed] = ros.internal.ros2.messages.ros2.default_type('uint8',1,0);
[data.status, info.status] = ros.internal.ros2.messages.diagnostic_msgs.diagnosticStatus;
info.status.MLdataType = 'struct';
info.status.MaxLen = NaN;
info.status.MinLen = 0;
info.MessageType = 'diagnostic_msgs/SelfTestResponse';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,14);
info.MatPath{1} = 'id';
info.MatPath{2} = 'passed';
info.MatPath{3} = 'status';
info.MatPath{4} = 'status.OK';
info.MatPath{5} = 'status.WARN';
info.MatPath{6} = 'status.ERROR';
info.MatPath{7} = 'status.STALE';
info.MatPath{8} = 'status.level';
info.MatPath{9} = 'status.name';
info.MatPath{10} = 'status.message';
info.MatPath{11} = 'status.hardware_id';
info.MatPath{12} = 'status.values';
info.MatPath{13} = 'status.values.key';
info.MatPath{14} = 'status.values.value';
