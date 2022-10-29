function [data, info] = getPlannerParamsRequest
%GetPlannerParams gives an empty data for moveit_msgs/GetPlannerParamsRequest

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'moveit_msgs/GetPlannerParamsRequest';
[data.PipelineId, info.PipelineId] = ros.internal.ros.messages.ros.char('string',0);
[data.PlannerConfig, info.PlannerConfig] = ros.internal.ros.messages.ros.char('string',0);
[data.Group, info.Group] = ros.internal.ros.messages.ros.char('string',0);
info.MessageType = 'moveit_msgs/GetPlannerParamsRequest';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,3);
info.MatPath{1} = 'pipeline_id';
info.MatPath{2} = 'planner_config';
info.MatPath{3} = 'group';
