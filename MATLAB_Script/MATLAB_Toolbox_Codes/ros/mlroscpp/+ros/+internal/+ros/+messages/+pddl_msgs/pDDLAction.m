function [data, info] = pDDLAction
%PDDLAction gives an empty data for pddl_msgs/PDDLAction

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'pddl_msgs/PDDLAction';
[data.Name, info.Name] = ros.internal.ros.messages.ros.char('string',0);
[data.Parameters, info.Parameters] = ros.internal.ros.messages.ros.char('string',0);
[data.Precondition, info.Precondition] = ros.internal.ros.messages.ros.char('string',0);
[data.Effect, info.Effect] = ros.internal.ros.messages.ros.char('string',0);
[data.OnCondition, info.OnCondition] = ros.internal.ros.messages.ros.char('string',0);
[data.ActionDuration, info.ActionDuration] = ros.internal.ros.messages.ros.char('string',0);
info.MessageType = 'pddl_msgs/PDDLAction';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,6);
info.MatPath{1} = 'name';
info.MatPath{2} = 'parameters';
info.MatPath{3} = 'precondition';
info.MatPath{4} = 'effect';
info.MatPath{5} = 'on_condition';
info.MatPath{6} = 'action_duration';
