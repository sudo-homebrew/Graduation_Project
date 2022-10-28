function [data, info] = loadLaunchRequest
%LoadLaunch gives an empty data for multimaster_msgs_fkie/LoadLaunchRequest

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'multimaster_msgs_fkie/LoadLaunchRequest';
[data.Package, info.Package] = ros.internal.ros.messages.ros.char('string',0);
[data.File, info.File] = ros.internal.ros.messages.ros.char('string',0);
[data.Argv, info.Argv] = ros.internal.ros.messages.ros.char('string',0);
info.MessageType = 'multimaster_msgs_fkie/LoadLaunchRequest';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,3);
info.MatPath{1} = 'package';
info.MatPath{2} = 'file';
info.MatPath{3} = 'argv';
