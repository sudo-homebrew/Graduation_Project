function [data, info] = setModelConfigurationRequest
%SetModelConfiguration gives an empty data for gazebo_msgs/SetModelConfigurationRequest

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'gazebo_msgs/SetModelConfigurationRequest';
[data.ModelName, info.ModelName] = ros.internal.ros.messages.ros.char('string',0);
[data.UrdfParamName, info.UrdfParamName] = ros.internal.ros.messages.ros.char('string',0);
[data.JointNames, info.JointNames] = ros.internal.ros.messages.ros.char('string',NaN);
[data.JointPositions, info.JointPositions] = ros.internal.ros.messages.ros.default_type('double',NaN);
info.MessageType = 'gazebo_msgs/SetModelConfigurationRequest';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,4);
info.MatPath{1} = 'model_name';
info.MatPath{2} = 'urdf_param_name';
info.MatPath{3} = 'joint_names';
info.MatPath{4} = 'joint_positions';
