function [data, info] = manualSelfTestResponse
%ManualSelfTest gives an empty data for sr_robot_msgs/ManualSelfTestResponse

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'sr_robot_msgs/ManualSelfTestResponse';
[data.Ok, info.Ok] = ros.internal.ros.messages.ros.default_type('logical',1);
[data.Message, info.Message] = ros.internal.ros.messages.ros.char('string',0);
info.MessageType = 'sr_robot_msgs/ManualSelfTestResponse';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,2);
info.MatPath{1} = 'ok';
info.MatPath{2} = 'message';
