function [data, info] = getPlannerParamsResponse
%GetPlannerParams gives an empty data for moveit_msgs/GetPlannerParamsResponse

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'moveit_msgs/GetPlannerParamsResponse';
[data.Params, info.Params] = ros.internal.ros.messages.moveit_msgs.plannerParams;
info.Params.MLdataType = 'struct';
info.MessageType = 'moveit_msgs/GetPlannerParamsResponse';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,4);
info.MatPath{1} = 'params';
info.MatPath{2} = 'params.keys';
info.MatPath{3} = 'params.values';
info.MatPath{4} = 'params.descriptions';
