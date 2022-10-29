function [data, info] = pDDLStep
%PDDLStep gives an empty data for pddl_msgs/PDDLStep

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'pddl_msgs/PDDLStep';
[data.Action, info.Action] = ros.internal.ros.messages.ros.char('string',0);
[data.Args, info.Args] = ros.internal.ros.messages.ros.char('string',NaN);
[data.StartTime, info.StartTime] = ros.internal.ros.messages.ros.char('string',0);
[data.ActionDuration, info.ActionDuration] = ros.internal.ros.messages.ros.char('string',0);
info.MessageType = 'pddl_msgs/PDDLStep';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,4);
info.MatPath{1} = 'action';
info.MatPath{2} = 'args';
info.MatPath{3} = 'start_time';
info.MatPath{4} = 'action_duration';
