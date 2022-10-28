function [data, info] = generateGraspsGoal
%GenerateGraspsGoal gives an empty data for cob_grasp_generation/GenerateGraspsGoal

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'cob_grasp_generation/GenerateGraspsGoal';
[data.ObjectName, info.ObjectName] = ros.internal.ros.messages.ros.char('string',0);
[data.GripperType, info.GripperType] = ros.internal.ros.messages.ros.char('string',0);
[data.Replan, info.Replan] = ros.internal.ros.messages.ros.default_type('logical',1);
[data.Viewer, info.Viewer] = ros.internal.ros.messages.ros.default_type('logical',1);
info.MessageType = 'cob_grasp_generation/GenerateGraspsGoal';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,4);
info.MatPath{1} = 'object_name';
info.MatPath{2} = 'gripper_type';
info.MatPath{3} = 'replan';
info.MatPath{4} = 'viewer';
