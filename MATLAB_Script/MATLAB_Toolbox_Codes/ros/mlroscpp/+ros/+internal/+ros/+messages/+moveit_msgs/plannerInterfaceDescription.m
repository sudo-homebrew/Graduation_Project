function [data, info] = plannerInterfaceDescription
%PlannerInterfaceDescription gives an empty data for moveit_msgs/PlannerInterfaceDescription

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'moveit_msgs/PlannerInterfaceDescription';
[data.Name, info.Name] = ros.internal.ros.messages.ros.char('string',0);
[data.PipelineId, info.PipelineId] = ros.internal.ros.messages.ros.char('string',0);
[data.PlannerIds, info.PlannerIds] = ros.internal.ros.messages.ros.char('string',NaN);
info.MessageType = 'moveit_msgs/PlannerInterfaceDescription';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,3);
info.MatPath{1} = 'name';
info.MatPath{2} = 'pipeline_id';
info.MatPath{3} = 'planner_ids';
