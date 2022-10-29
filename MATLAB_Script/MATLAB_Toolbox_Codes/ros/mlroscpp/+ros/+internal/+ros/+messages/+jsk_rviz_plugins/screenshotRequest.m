function [data, info] = screenshotRequest
%Screenshot gives an empty data for jsk_rviz_plugins/ScreenshotRequest

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'jsk_rviz_plugins/ScreenshotRequest';
[data.FileName, info.FileName] = ros.internal.ros.messages.ros.char('string',0);
info.MessageType = 'jsk_rviz_plugins/ScreenshotRequest';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,1);
info.MatPath{1} = 'file_name';
