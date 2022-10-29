function [data, info] = smachContainerStatus
%SmachContainerStatus gives an empty data for smach_msgs/SmachContainerStatus

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'smach_msgs/SmachContainerStatus';
[data.Header, info.Header] = ros.internal.ros.messages.std_msgs.header;
info.Header.MLdataType = 'struct';
[data.Path, info.Path] = ros.internal.ros.messages.ros.char('string',0);
[data.InitialStates, info.InitialStates] = ros.internal.ros.messages.ros.char('string',NaN);
[data.ActiveStates, info.ActiveStates] = ros.internal.ros.messages.ros.char('string',NaN);
[data.LocalData, info.LocalData] = ros.internal.ros.messages.ros.char('string',0);
[data.Info, info.Info] = ros.internal.ros.messages.ros.char('string',0);
info.MessageType = 'smach_msgs/SmachContainerStatus';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,11);
info.MatPath{1} = 'header';
info.MatPath{2} = 'header.seq';
info.MatPath{3} = 'header.stamp';
info.MatPath{4} = 'header.stamp.sec';
info.MatPath{5} = 'header.stamp.nsec';
info.MatPath{6} = 'header.frame_id';
info.MatPath{7} = 'path';
info.MatPath{8} = 'initial_states';
info.MatPath{9} = 'active_states';
info.MatPath{10} = 'local_data';
info.MatPath{11} = 'info';
