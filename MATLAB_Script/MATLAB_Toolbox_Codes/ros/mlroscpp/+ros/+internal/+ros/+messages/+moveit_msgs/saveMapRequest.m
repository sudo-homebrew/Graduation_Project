function [data, info] = saveMapRequest
%SaveMap gives an empty data for moveit_msgs/SaveMapRequest

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'moveit_msgs/SaveMapRequest';
[data.Filename, info.Filename] = ros.internal.ros.messages.ros.char('string',0);
info.MessageType = 'moveit_msgs/SaveMapRequest';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,1);
info.MatPath{1} = 'filename';
