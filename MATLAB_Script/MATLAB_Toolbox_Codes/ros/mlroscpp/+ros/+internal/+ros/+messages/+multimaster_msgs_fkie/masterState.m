function [data, info] = masterState
%MasterState gives an empty data for multimaster_msgs_fkie/MasterState

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'multimaster_msgs_fkie/MasterState';
[data.State, info.State] = ros.internal.ros.messages.ros.char('string',0);
[data.Master, info.Master] = ros.internal.ros.messages.multimaster_msgs_fkie.rOSMaster;
info.Master.MLdataType = 'struct';
[data.STATENEW, info.STATENEW] = ros.internal.ros.messages.ros.char('string',0);
[data.STATENEW, info.STATENEW] = ros.internal.ros.messages.ros.char('string',1,'new');
[data.STATEREMOVED, info.STATEREMOVED] = ros.internal.ros.messages.ros.char('string',0);
[data.STATEREMOVED, info.STATEREMOVED] = ros.internal.ros.messages.ros.char('string',1,'removed');
[data.STATECHANGED, info.STATECHANGED] = ros.internal.ros.messages.ros.char('string',0);
[data.STATECHANGED, info.STATECHANGED] = ros.internal.ros.messages.ros.char('string',1,'changed');
info.MessageType = 'multimaster_msgs_fkie/MasterState';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,12);
info.MatPath{1} = 'state';
info.MatPath{2} = 'master';
info.MatPath{3} = 'master.name';
info.MatPath{4} = 'master.uri';
info.MatPath{5} = 'master.timestamp';
info.MatPath{6} = 'master.timestamp_local';
info.MatPath{7} = 'master.online';
info.MatPath{8} = 'master.discoverer_name';
info.MatPath{9} = 'master.monitoruri';
info.MatPath{10} = 'STATE_NEW';
info.MatPath{11} = 'STATE_REMOVED';
info.MatPath{12} = 'STATE_CHANGED';
