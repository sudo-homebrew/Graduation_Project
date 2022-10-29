function [data, info] = taskRequest
%Task gives an empty data for multimaster_msgs_fkie/TaskRequest

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'multimaster_msgs_fkie/TaskRequest';
[data.Node, info.Node] = ros.internal.ros.messages.ros.char('string',0);
info.MessageType = 'multimaster_msgs_fkie/TaskRequest';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,1);
info.MatPath{1} = 'node';
