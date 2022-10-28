function [data, info] = changeControlDimensionsRequest
%ChangeControlDimensions gives an empty data for moveit_msgs/ChangeControlDimensionsRequest

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'moveit_msgs/ChangeControlDimensionsRequest';
[data.ControlXTranslation, info.ControlXTranslation] = ros.internal.ros.messages.ros.default_type('logical',1);
[data.ControlYTranslation, info.ControlYTranslation] = ros.internal.ros.messages.ros.default_type('logical',1);
[data.ControlZTranslation, info.ControlZTranslation] = ros.internal.ros.messages.ros.default_type('logical',1);
[data.ControlXRotation, info.ControlXRotation] = ros.internal.ros.messages.ros.default_type('logical',1);
[data.ControlYRotation, info.ControlYRotation] = ros.internal.ros.messages.ros.default_type('logical',1);
[data.ControlZRotation, info.ControlZRotation] = ros.internal.ros.messages.ros.default_type('logical',1);
info.MessageType = 'moveit_msgs/ChangeControlDimensionsRequest';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,6);
info.MatPath{1} = 'control_x_translation';
info.MatPath{2} = 'control_y_translation';
info.MatPath{3} = 'control_z_translation';
info.MatPath{4} = 'control_x_rotation';
info.MatPath{5} = 'control_y_rotation';
info.MatPath{6} = 'control_z_rotation';
