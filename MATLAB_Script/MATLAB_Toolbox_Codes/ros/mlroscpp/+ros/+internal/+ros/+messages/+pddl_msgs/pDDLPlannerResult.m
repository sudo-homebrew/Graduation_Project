function [data, info] = pDDLPlannerResult
%PDDLPlannerResult gives an empty data for pddl_msgs/PDDLPlannerResult

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'pddl_msgs/PDDLPlannerResult';
[data.Data, info.Data] = ros.internal.ros.messages.ros.char('string',NaN);
[data.UseDurativeAction, info.UseDurativeAction] = ros.internal.ros.messages.ros.default_type('logical',1);
[data.Sequence, info.Sequence] = ros.internal.ros.messages.pddl_msgs.pDDLStep;
info.Sequence.MLdataType = 'struct';
info.Sequence.MaxLen = NaN;
info.Sequence.MinLen = 0;
data.Sequence = data.Sequence([],1);
info.MessageType = 'pddl_msgs/PDDLPlannerResult';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,7);
info.MatPath{1} = 'data';
info.MatPath{2} = 'use_durative_action';
info.MatPath{3} = 'sequence';
info.MatPath{4} = 'sequence.action';
info.MatPath{5} = 'sequence.args';
info.MatPath{6} = 'sequence.start_time';
info.MatPath{7} = 'sequence.action_duration';
