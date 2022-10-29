function [data, info] = pDDLProblem
%PDDLProblem gives an empty data for pddl_msgs/PDDLProblem

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'pddl_msgs/PDDLProblem';
[data.Name, info.Name] = ros.internal.ros.messages.ros.char('string',0);
[data.Domain, info.Domain] = ros.internal.ros.messages.ros.char('string',0);
[data.Objects, info.Objects] = ros.internal.ros.messages.pddl_msgs.pDDLObject;
info.Objects.MLdataType = 'struct';
info.Objects.MaxLen = NaN;
info.Objects.MinLen = 0;
data.Objects = data.Objects([],1);
[data.Initial, info.Initial] = ros.internal.ros.messages.ros.char('string',NaN);
[data.Goal, info.Goal] = ros.internal.ros.messages.ros.char('string',0);
[data.Metric, info.Metric] = ros.internal.ros.messages.ros.char('string',0);
info.MessageType = 'pddl_msgs/PDDLProblem';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,8);
info.MatPath{1} = 'name';
info.MatPath{2} = 'domain';
info.MatPath{3} = 'objects';
info.MatPath{4} = 'objects.name';
info.MatPath{5} = 'objects.type';
info.MatPath{6} = 'initial';
info.MatPath{7} = 'goal';
info.MatPath{8} = 'metric';
