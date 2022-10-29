function [data, info] = showGraspsGoal
%ShowGraspsGoal gives an empty data for cob_grasp_generation/ShowGraspsGoal

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'cob_grasp_generation/ShowGraspsGoal';
[data.ObjectName, info.ObjectName] = ros.internal.ros.messages.ros.char('string',0);
[data.GripperType, info.GripperType] = ros.internal.ros.messages.ros.char('string',0);
[data.GripperSide, info.GripperSide] = ros.internal.ros.messages.ros.char('string',0);
[data.GraspId, info.GraspId] = ros.internal.ros.messages.ros.default_type('int32',1);
[data.SortByQuality, info.SortByQuality] = ros.internal.ros.messages.ros.default_type('logical',1);
info.MessageType = 'cob_grasp_generation/ShowGraspsGoal';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,5);
info.MatPath{1} = 'object_name';
info.MatPath{2} = 'gripper_type';
info.MatPath{3} = 'gripper_side';
info.MatPath{4} = 'grasp_id';
info.MatPath{5} = 'sort_by_quality';
