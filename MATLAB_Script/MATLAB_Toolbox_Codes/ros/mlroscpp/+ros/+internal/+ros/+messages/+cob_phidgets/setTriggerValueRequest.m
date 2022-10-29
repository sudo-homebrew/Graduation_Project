function [data, info] = setTriggerValueRequest
%SetTriggerValue gives an empty data for cob_phidgets/SetTriggerValueRequest

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'cob_phidgets/SetTriggerValueRequest';
[data.Index, info.Index] = ros.internal.ros.messages.ros.default_type('int8',1);
[data.TriggerValue, info.TriggerValue] = ros.internal.ros.messages.ros.default_type('uint16',1);
info.MessageType = 'cob_phidgets/SetTriggerValueRequest';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,2);
info.MatPath{1} = 'index';
info.MatPath{2} = 'trigger_value';
