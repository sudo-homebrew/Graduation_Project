function [data, info] = triggerPublishGoal
%TriggerPublishGoal gives an empty data for bride_tutorials/TriggerPublishGoal

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'bride_tutorials/TriggerPublishGoal';
[data.Test, info.Test] = ros.internal.ros.messages.ros.default_type('uint32',1);
info.MessageType = 'bride_tutorials/TriggerPublishGoal';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,1);
info.MatPath{1} = 'test';
