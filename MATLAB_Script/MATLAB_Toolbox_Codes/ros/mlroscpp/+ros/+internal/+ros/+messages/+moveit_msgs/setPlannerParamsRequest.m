function [data, info] = setPlannerParamsRequest
%SetPlannerParams gives an empty data for moveit_msgs/SetPlannerParamsRequest

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'moveit_msgs/SetPlannerParamsRequest';
[data.PipelineId, info.PipelineId] = ros.internal.ros.messages.ros.char('string',0);
[data.PlannerConfig, info.PlannerConfig] = ros.internal.ros.messages.ros.char('string',0);
[data.Group, info.Group] = ros.internal.ros.messages.ros.char('string',0);
[data.Params, info.Params] = ros.internal.ros.messages.moveit_msgs.plannerParams;
info.Params.MLdataType = 'struct';
[data.Replace, info.Replace] = ros.internal.ros.messages.ros.default_type('logical',1);
info.MessageType = 'moveit_msgs/SetPlannerParamsRequest';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,8);
info.MatPath{1} = 'pipeline_id';
info.MatPath{2} = 'planner_config';
info.MatPath{3} = 'group';
info.MatPath{4} = 'params';
info.MatPath{5} = 'params.keys';
info.MatPath{6} = 'params.values';
info.MatPath{7} = 'params.descriptions';
info.MatPath{8} = 'replace';
