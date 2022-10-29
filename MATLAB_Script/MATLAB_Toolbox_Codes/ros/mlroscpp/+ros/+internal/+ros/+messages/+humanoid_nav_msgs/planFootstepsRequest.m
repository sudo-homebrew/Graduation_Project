function [data, info] = planFootstepsRequest
%PlanFootsteps gives an empty data for humanoid_nav_msgs/PlanFootstepsRequest

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'humanoid_nav_msgs/PlanFootstepsRequest';
[data.Start, info.Start] = ros.internal.ros.messages.geometry_msgs.pose2D;
info.Start.MLdataType = 'struct';
[data.Goal, info.Goal] = ros.internal.ros.messages.geometry_msgs.pose2D;
info.Goal.MLdataType = 'struct';
info.MessageType = 'humanoid_nav_msgs/PlanFootstepsRequest';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,8);
info.MatPath{1} = 'start';
info.MatPath{2} = 'start.x';
info.MatPath{3} = 'start.y';
info.MatPath{4} = 'start.theta';
info.MatPath{5} = 'goal';
info.MatPath{6} = 'goal.x';
info.MatPath{7} = 'goal.y';
info.MatPath{8} = 'goal.theta';
