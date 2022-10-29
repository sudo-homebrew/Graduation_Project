function [data, info] = masterState
%MasterState gives an empty data for fkie_multimaster_msgs/MasterState

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'fkie_multimaster_msgs/MasterState';
[data.State, info.State] = ros.internal.ros.messages.ros.char('string',0);
[data.Master, info.Master] = ros.internal.ros.messages.fkie_multimaster_msgs.rOSMaster;
info.Master.MLdataType = 'struct';
[data.STATENEW, info.STATENEW] = ros.internal.ros.messages.ros.char('string',0);
[data.STATENEW, info.STATENEW] = ros.internal.ros.messages.ros.char('string',1,'new');
[data.STATEREMOVED, info.STATEREMOVED] = ros.internal.ros.messages.ros.char('string',0);
[data.STATEREMOVED, info.STATEREMOVED] = ros.internal.ros.messages.ros.char('string',1,'removed');
[data.STATECHANGED, info.STATECHANGED] = ros.internal.ros.messages.ros.char('string',0);
[data.STATECHANGED, info.STATECHANGED] = ros.internal.ros.messages.ros.char('string',1,'changed');
info.MessageType = 'fkie_multimaster_msgs/MasterState';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,16);
info.MatPath{1} = 'state';
info.MatPath{2} = 'master';
info.MatPath{3} = 'master.name';
info.MatPath{4} = 'master.uri';
info.MatPath{5} = 'master.last_change';
info.MatPath{6} = 'master.last_change.sec';
info.MatPath{7} = 'master.last_change.nsec';
info.MatPath{8} = 'master.last_change_local';
info.MatPath{9} = 'master.last_change_local.sec';
info.MatPath{10} = 'master.last_change_local.nsec';
info.MatPath{11} = 'master.online';
info.MatPath{12} = 'master.discoverer_name';
info.MatPath{13} = 'master.monitoruri';
info.MatPath{14} = 'STATE_NEW';
info.MatPath{15} = 'STATE_REMOVED';
info.MatPath{16} = 'STATE_CHANGED';
