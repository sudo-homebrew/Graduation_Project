function [data, info] = pDDLDomain
%PDDLDomain gives an empty data for pddl_msgs/PDDLDomain

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'pddl_msgs/PDDLDomain';
[data.Name, info.Name] = ros.internal.ros.messages.ros.char('string',0);
[data.Requirements, info.Requirements] = ros.internal.ros.messages.ros.char('string',0);
[data.Types, info.Types] = ros.internal.ros.messages.ros.char('string',NaN);
[data.Constants, info.Constants] = ros.internal.ros.messages.ros.char('string',NaN);
[data.Predicates, info.Predicates] = ros.internal.ros.messages.ros.char('string',NaN);
[data.Actions, info.Actions] = ros.internal.ros.messages.pddl_msgs.pDDLAction;
info.Actions.MLdataType = 'struct';
info.Actions.MaxLen = NaN;
info.Actions.MinLen = 0;
data.Actions = data.Actions([],1);
[data.Functions, info.Functions] = ros.internal.ros.messages.ros.char('string',NaN);
info.MessageType = 'pddl_msgs/PDDLDomain';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,13);
info.MatPath{1} = 'name';
info.MatPath{2} = 'requirements';
info.MatPath{3} = 'types';
info.MatPath{4} = 'constants';
info.MatPath{5} = 'predicates';
info.MatPath{6} = 'actions';
info.MatPath{7} = 'actions.name';
info.MatPath{8} = 'actions.parameters';
info.MatPath{9} = 'actions.precondition';
info.MatPath{10} = 'actions.effect';
info.MatPath{11} = 'actions.on_condition';
info.MatPath{12} = 'actions.action_duration';
info.MatPath{13} = 'functions';
