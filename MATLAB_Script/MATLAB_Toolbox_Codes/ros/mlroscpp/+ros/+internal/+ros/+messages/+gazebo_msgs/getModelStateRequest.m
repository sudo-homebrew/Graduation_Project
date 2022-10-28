function [data, info] = getModelStateRequest
%GetModelState gives an empty data for gazebo_msgs/GetModelStateRequest

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'gazebo_msgs/GetModelStateRequest';
[data.ModelName, info.ModelName] = ros.internal.ros.messages.ros.char('string',0);
[data.RelativeEntityName, info.RelativeEntityName] = ros.internal.ros.messages.ros.char('string',0);
info.MessageType = 'gazebo_msgs/GetModelStateRequest';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,2);
info.MatPath{1} = 'model_name';
info.MatPath{2} = 'relative_entity_name';
