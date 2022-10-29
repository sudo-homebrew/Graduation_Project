function [data, info] = targetObjRequest
%TargetObj gives an empty data for posedetection_msgs/TargetObjRequest

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'posedetection_msgs/TargetObjRequest';
[data.Type, info.Type] = ros.internal.ros.messages.ros.char('string',0);
info.MessageType = 'posedetection_msgs/TargetObjRequest';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,1);
info.MatPath{1} = 'type';
