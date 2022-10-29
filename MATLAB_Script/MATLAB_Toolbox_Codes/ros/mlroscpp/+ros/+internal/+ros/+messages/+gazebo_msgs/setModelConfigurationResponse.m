function [data, info] = setModelConfigurationResponse
%SetModelConfiguration gives an empty data for gazebo_msgs/SetModelConfigurationResponse

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'gazebo_msgs/SetModelConfigurationResponse';
[data.Success, info.Success] = ros.internal.ros.messages.ros.default_type('logical',1);
[data.StatusMessage, info.StatusMessage] = ros.internal.ros.messages.ros.char('string',0);
info.MessageType = 'gazebo_msgs/SetModelConfigurationResponse';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,2);
info.MatPath{1} = 'success';
info.MatPath{2} = 'status_message';
