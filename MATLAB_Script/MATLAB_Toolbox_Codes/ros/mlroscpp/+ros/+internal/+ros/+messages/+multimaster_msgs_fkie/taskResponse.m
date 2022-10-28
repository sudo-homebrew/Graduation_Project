function [data, info] = taskResponse
%Task gives an empty data for multimaster_msgs_fkie/TaskResponse

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'multimaster_msgs_fkie/TaskResponse';
info.MessageType = 'multimaster_msgs_fkie/TaskResponse';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,0);
