function [data, info] = smachContainerInitialStatusCmd
%SmachContainerInitialStatusCmd gives an empty data for smach_msgs/SmachContainerInitialStatusCmd

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'smach_msgs/SmachContainerInitialStatusCmd';
[data.Path, info.Path] = ros.internal.ros.messages.ros.char('string',0);
[data.InitialStates, info.InitialStates] = ros.internal.ros.messages.ros.char('string',NaN);
[data.LocalData, info.LocalData] = ros.internal.ros.messages.ros.char('string',0);
info.MessageType = 'smach_msgs/SmachContainerInitialStatusCmd';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,3);
info.MatPath{1} = 'path';
info.MatPath{2} = 'initial_states';
info.MatPath{3} = 'local_data';
