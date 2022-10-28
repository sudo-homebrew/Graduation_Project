function [data, info] = planFootstepsBetweenFeetResponse
%PlanFootstepsBetweenFeet gives an empty data for humanoid_nav_msgs/PlanFootstepsBetweenFeetResponse

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'humanoid_nav_msgs/PlanFootstepsBetweenFeetResponse';
[data.Result, info.Result] = ros.internal.ros.messages.ros.default_type('logical',1);
[data.Footsteps, info.Footsteps] = ros.internal.ros.messages.humanoid_nav_msgs.stepTarget;
info.Footsteps.MLdataType = 'struct';
info.Footsteps.MaxLen = NaN;
info.Footsteps.MinLen = 0;
data.Footsteps = data.Footsteps([],1);
[data.Costs, info.Costs] = ros.internal.ros.messages.ros.default_type('double',1);
[data.FinalEps, info.FinalEps] = ros.internal.ros.messages.ros.default_type('double',1);
[data.PlanningTime, info.PlanningTime] = ros.internal.ros.messages.ros.default_type('double',1);
[data.ExpandedStates, info.ExpandedStates] = ros.internal.ros.messages.ros.default_type('int64',1);
info.MessageType = 'humanoid_nav_msgs/PlanFootstepsBetweenFeetResponse';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,13);
info.MatPath{1} = 'result';
info.MatPath{2} = 'footsteps';
info.MatPath{3} = 'footsteps.pose';
info.MatPath{4} = 'footsteps.pose.x';
info.MatPath{5} = 'footsteps.pose.y';
info.MatPath{6} = 'footsteps.pose.theta';
info.MatPath{7} = 'footsteps.leg';
info.MatPath{8} = 'footsteps.right';
info.MatPath{9} = 'footsteps.left';
info.MatPath{10} = 'costs';
info.MatPath{11} = 'final_eps';
info.MatPath{12} = 'planning_time';
info.MatPath{13} = 'expanded_states';
