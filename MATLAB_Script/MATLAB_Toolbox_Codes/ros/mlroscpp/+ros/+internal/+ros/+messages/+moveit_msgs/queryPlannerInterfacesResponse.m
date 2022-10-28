function [data, info] = queryPlannerInterfacesResponse
%QueryPlannerInterfaces gives an empty data for moveit_msgs/QueryPlannerInterfacesResponse

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'moveit_msgs/QueryPlannerInterfacesResponse';
[data.PlannerInterfaces, info.PlannerInterfaces] = ros.internal.ros.messages.moveit_msgs.plannerInterfaceDescription;
info.PlannerInterfaces.MLdataType = 'struct';
info.PlannerInterfaces.MaxLen = NaN;
info.PlannerInterfaces.MinLen = 0;
data.PlannerInterfaces = data.PlannerInterfaces([],1);
info.MessageType = 'moveit_msgs/QueryPlannerInterfacesResponse';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,4);
info.MatPath{1} = 'planner_interfaces';
info.MatPath{2} = 'planner_interfaces.name';
info.MatPath{3} = 'planner_interfaces.pipeline_id';
info.MatPath{4} = 'planner_interfaces.planner_ids';
