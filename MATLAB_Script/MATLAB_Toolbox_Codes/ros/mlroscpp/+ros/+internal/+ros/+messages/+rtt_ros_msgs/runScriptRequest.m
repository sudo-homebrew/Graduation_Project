function [data, info] = runScriptRequest
%RunScript gives an empty data for rtt_ros_msgs/RunScriptRequest

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'rtt_ros_msgs/RunScriptRequest';
[data.FilePath, info.FilePath] = ros.internal.ros.messages.ros.char('string',0);
info.MessageType = 'rtt_ros_msgs/RunScriptRequest';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,1);
info.MatPath{1} = 'file_path';
