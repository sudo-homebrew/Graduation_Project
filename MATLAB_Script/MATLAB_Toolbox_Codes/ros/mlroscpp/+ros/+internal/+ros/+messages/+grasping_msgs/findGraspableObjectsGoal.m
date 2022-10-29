function [data, info] = findGraspableObjectsGoal
%FindGraspableObjectsGoal gives an empty data for grasping_msgs/FindGraspableObjectsGoal

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'grasping_msgs/FindGraspableObjectsGoal';
[data.PlanGrasps, info.PlanGrasps] = ros.internal.ros.messages.ros.default_type('logical',1);
info.MessageType = 'grasping_msgs/FindGraspableObjectsGoal';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,1);
info.MatPath{1} = 'plan_grasps';
