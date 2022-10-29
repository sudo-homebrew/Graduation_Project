function [data, info] = groupState
%GroupState gives an empty data for dynamic_reconfigure/GroupState

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'dynamic_reconfigure/GroupState';
[data.Name, info.Name] = ros.internal.ros.messages.ros.char('string',0);
[data.State, info.State] = ros.internal.ros.messages.ros.default_type('logical',1);
[data.Id, info.Id] = ros.internal.ros.messages.ros.default_type('int32',1);
[data.Parent, info.Parent] = ros.internal.ros.messages.ros.default_type('int32',1);
info.MessageType = 'dynamic_reconfigure/GroupState';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,4);
info.MatPath{1} = 'name';
info.MatPath{2} = 'state';
info.MatPath{3} = 'id';
info.MatPath{4} = 'parent';
