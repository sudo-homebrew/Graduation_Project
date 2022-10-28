function [data, info] = selfTestResponse
%SelfTest gives an empty data for diagnostic_msgs/SelfTestResponse

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'diagnostic_msgs/SelfTestResponse';
[data.Id, info.Id] = ros.internal.ros.messages.ros.char('string',0);
[data.Passed, info.Passed] = ros.internal.ros.messages.ros.default_type('int8',1);
[data.Status, info.Status] = ros.internal.ros.messages.diagnostic_msgs.diagnosticStatus;
info.Status.MLdataType = 'struct';
info.Status.MaxLen = NaN;
info.Status.MinLen = 0;
data.Status = data.Status([],1);
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
