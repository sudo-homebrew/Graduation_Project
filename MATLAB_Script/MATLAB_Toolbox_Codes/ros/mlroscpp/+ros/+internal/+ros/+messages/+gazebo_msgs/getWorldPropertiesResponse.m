function [data, info] = getWorldPropertiesResponse
%GetWorldProperties gives an empty data for gazebo_msgs/GetWorldPropertiesResponse

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'gazebo_msgs/GetWorldPropertiesResponse';
[data.SimTime, info.SimTime] = ros.internal.ros.messages.ros.default_type('double',1);
[data.ModelNames, info.ModelNames] = ros.internal.ros.messages.ros.char('string',NaN);
[data.RenderingEnabled, info.RenderingEnabled] = ros.internal.ros.messages.ros.default_type('logical',1);
[data.Success, info.Success] = ros.internal.ros.messages.ros.default_type('logical',1);
[data.StatusMessage, info.StatusMessage] = ros.internal.ros.messages.ros.char('string',0);
info.MessageType = 'gazebo_msgs/GetWorldPropertiesResponse';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,5);
info.MatPath{1} = 'sim_time';
info.MatPath{2} = 'model_names';
info.MatPath{3} = 'rendering_enabled';
info.MatPath{4} = 'success';
info.MatPath{5} = 'status_message';
