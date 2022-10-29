function [data, info] = manualSelfTestRequest
%ManualSelfTest gives an empty data for sr_robot_msgs/ManualSelfTestRequest

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'sr_robot_msgs/ManualSelfTestRequest';
[data.Message, info.Message] = ros.internal.ros.messages.ros.char('string',0);
info.MessageType = 'sr_robot_msgs/ManualSelfTestRequest';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,1);
info.MatPath{1} = 'message';
