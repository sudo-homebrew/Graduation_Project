function [data, info] = subState
%SubState gives an empty data for robotnik_msgs/SubState

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'robotnik_msgs/SubState';
[data.Substate, info.Substate] = ros.internal.ros.messages.ros.default_type('int32',1);
[data.SubstateDescription, info.SubstateDescription] = ros.internal.ros.messages.ros.char('string',0);
[data.Msg, info.Msg] = ros.internal.ros.messages.ros.char('string',0);
info.MessageType = 'robotnik_msgs/SubState';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,3);
info.MatPath{1} = 'substate';
info.MatPath{2} = 'substate_description';
info.MatPath{3} = 'msg';
