function [data, info] = listCamerasResponse
%ListCameras gives an empty data for baxter_core_msgs/ListCamerasResponse

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'baxter_core_msgs/ListCamerasResponse';
[data.Cameras, info.Cameras] = ros.internal.ros.messages.ros.char('string',NaN);
info.MessageType = 'baxter_core_msgs/ListCamerasResponse';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,1);
info.MatPath{1} = 'cameras';
