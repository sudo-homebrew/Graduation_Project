function [data, info] = pDDLActionArray
%PDDLActionArray gives an empty data for pddl_msgs/PDDLActionArray

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'pddl_msgs/PDDLActionArray';
[data.Actions, info.Actions] = ros.internal.ros.messages.pddl_msgs.pDDLAction;
info.Actions.MLdataType = 'struct';
info.Actions.MaxLen = NaN;
info.Actions.MinLen = 0;
data.Actions = data.Actions([],1);
info.MessageType = 'pddl_msgs/PDDLActionArray';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,7);
info.MatPath{1} = 'actions';
info.MatPath{2} = 'actions.name';
info.MatPath{3} = 'actions.parameters';
info.MatPath{4} = 'actions.precondition';
info.MatPath{5} = 'actions.effect';
info.MatPath{6} = 'actions.on_condition';
info.MatPath{7} = 'actions.action_duration';
