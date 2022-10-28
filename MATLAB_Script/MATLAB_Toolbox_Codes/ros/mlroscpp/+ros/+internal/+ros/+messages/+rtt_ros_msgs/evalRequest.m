function [data, info] = evalRequest
%Eval gives an empty data for rtt_ros_msgs/EvalRequest

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'rtt_ros_msgs/EvalRequest';
[data.Code, info.Code] = ros.internal.ros.messages.ros.char('string',0);
info.MessageType = 'rtt_ros_msgs/EvalRequest';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,1);
info.MatPath{1} = 'code';
