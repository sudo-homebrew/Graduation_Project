function [data, info] = loadLaunchRequest
%LoadLaunch gives an empty data for fkie_multimaster_msgs/LoadLaunchRequest

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'fkie_multimaster_msgs/LoadLaunchRequest';
[data.Path, info.Path] = ros.internal.ros.messages.ros.char('string',0);
info.MessageType = 'fkie_multimaster_msgs/LoadLaunchRequest';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,1);
info.MatPath{1} = 'path';
