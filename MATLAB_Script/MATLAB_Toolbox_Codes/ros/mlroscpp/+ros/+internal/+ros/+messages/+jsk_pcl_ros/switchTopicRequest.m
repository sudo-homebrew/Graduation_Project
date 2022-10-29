function [data, info] = switchTopicRequest
%SwitchTopic gives an empty data for jsk_pcl_ros/SwitchTopicRequest

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'jsk_pcl_ros/SwitchTopicRequest';
[data.CameraInfo, info.CameraInfo] = ros.internal.ros.messages.ros.char('string',0);
[data.Points, info.Points] = ros.internal.ros.messages.ros.char('string',0);
info.MessageType = 'jsk_pcl_ros/SwitchTopicRequest';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,2);
info.MatPath{1} = 'camera_info';
info.MatPath{2} = 'points';
