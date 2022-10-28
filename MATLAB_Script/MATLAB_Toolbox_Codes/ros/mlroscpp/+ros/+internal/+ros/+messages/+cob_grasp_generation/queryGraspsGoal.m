function [data, info] = queryGraspsGoal
%QueryGraspsGoal gives an empty data for cob_grasp_generation/QueryGraspsGoal

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'cob_grasp_generation/QueryGraspsGoal';
[data.ObjectName, info.ObjectName] = ros.internal.ros.messages.ros.char('string',0);
[data.GripperType, info.GripperType] = ros.internal.ros.messages.ros.char('string',0);
[data.GripperSide, info.GripperSide] = ros.internal.ros.messages.ros.char('string',0);
[data.GraspId, info.GraspId] = ros.internal.ros.messages.ros.default_type('uint32',1);
[data.NumGrasps, info.NumGrasps] = ros.internal.ros.messages.ros.default_type('uint32',1);
[data.Threshold, info.Threshold] = ros.internal.ros.messages.ros.default_type('double',1);
info.MessageType = 'cob_grasp_generation/QueryGraspsGoal';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,6);
info.MatPath{1} = 'object_name';
info.MatPath{2} = 'gripper_type';
info.MatPath{3} = 'gripper_side';
info.MatPath{4} = 'grasp_id';
info.MatPath{5} = 'num_grasps';
info.MatPath{6} = 'threshold';
