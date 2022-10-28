function [data, info] = taskRequest
%Task gives an empty data for fkie_multimaster_msgs/TaskRequest

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'fkie_multimaster_msgs/TaskRequest';
[data.Node, info.Node] = ros.internal.ros.messages.ros.char('string',0);
info.MessageType = 'fkie_multimaster_msgs/TaskRequest';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,1);
info.MatPath{1} = 'node';
