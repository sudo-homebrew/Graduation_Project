function [data, info] = getModelPropertiesResponse
%GetModelProperties gives an empty data for gazebo_msgs/GetModelPropertiesResponse

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'gazebo_msgs/GetModelPropertiesResponse';
[data.ParentModelName, info.ParentModelName] = ros.internal.ros.messages.ros.char('string',0);
[data.CanonicalBodyName, info.CanonicalBodyName] = ros.internal.ros.messages.ros.char('string',0);
[data.BodyNames, info.BodyNames] = ros.internal.ros.messages.ros.char('string',NaN);
[data.GeomNames, info.GeomNames] = ros.internal.ros.messages.ros.char('string',NaN);
[data.JointNames, info.JointNames] = ros.internal.ros.messages.ros.char('string',NaN);
[data.ChildModelNames, info.ChildModelNames] = ros.internal.ros.messages.ros.char('string',NaN);
[data.IsStatic, info.IsStatic] = ros.internal.ros.messages.ros.default_type('logical',1);
[data.Success, info.Success] = ros.internal.ros.messages.ros.default_type('logical',1);
[data.StatusMessage, info.StatusMessage] = ros.internal.ros.messages.ros.char('string',0);
info.MessageType = 'gazebo_msgs/GetModelPropertiesResponse';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,9);
info.MatPath{1} = 'parent_model_name';
info.MatPath{2} = 'canonical_body_name';
info.MatPath{3} = 'body_names';
info.MatPath{4} = 'geom_names';
info.MatPath{5} = 'joint_names';
info.MatPath{6} = 'child_model_names';
info.MatPath{7} = 'is_static';
info.MatPath{8} = 'success';
info.MatPath{9} = 'status_message';
