function [data, info] = deleteLightRequest
%DeleteLight gives an empty data for gazebo_msgs/DeleteLightRequest

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'gazebo_msgs/DeleteLightRequest';
[data.LightName, info.LightName] = ros.internal.ros.messages.ros.char('string',0);
info.MessageType = 'gazebo_msgs/DeleteLightRequest';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,1);
info.MatPath{1} = 'light_name';
