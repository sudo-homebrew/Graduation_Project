function [data, info] = applyPlanningSceneResponse
%ApplyPlanningScene gives an empty data for moveit_msgs/ApplyPlanningSceneResponse

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'moveit_msgs/ApplyPlanningSceneResponse';
[data.Success, info.Success] = ros.internal.ros.messages.ros.default_type('logical',1);
info.MessageType = 'moveit_msgs/ApplyPlanningSceneResponse';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,1);
info.MatPath{1} = 'success';
