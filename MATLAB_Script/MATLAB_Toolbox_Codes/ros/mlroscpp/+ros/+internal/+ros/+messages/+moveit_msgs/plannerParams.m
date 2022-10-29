function [data, info] = plannerParams
%PlannerParams gives an empty data for moveit_msgs/PlannerParams

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'moveit_msgs/PlannerParams';
[data.Keys, info.Keys] = ros.internal.ros.messages.ros.char('string',NaN);
[data.Values, info.Values] = ros.internal.ros.messages.ros.char('string',NaN);
[data.Descriptions, info.Descriptions] = ros.internal.ros.messages.ros.char('string',NaN);
info.MessageType = 'moveit_msgs/PlannerParams';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,3);
info.MatPath{1} = 'keys';
info.MatPath{2} = 'values';
info.MatPath{3} = 'descriptions';
