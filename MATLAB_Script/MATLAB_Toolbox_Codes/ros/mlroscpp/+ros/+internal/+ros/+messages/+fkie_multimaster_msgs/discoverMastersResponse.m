function [data, info] = discoverMastersResponse
%DiscoverMasters gives an empty data for fkie_multimaster_msgs/DiscoverMastersResponse

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'fkie_multimaster_msgs/DiscoverMastersResponse';
[data.Masters, info.Masters] = ros.internal.ros.messages.fkie_multimaster_msgs.rOSMaster;
info.Masters.MLdataType = 'struct';
info.Masters.MaxLen = NaN;
info.Masters.MinLen = 0;
data.Masters = data.Masters([],1);
info.MessageType = 'fkie_multimaster_msgs/DiscoverMastersResponse';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,12);
info.MatPath{1} = 'masters';
info.MatPath{2} = 'masters.name';
info.MatPath{3} = 'masters.uri';
info.MatPath{4} = 'masters.last_change';
info.MatPath{5} = 'masters.last_change.sec';
info.MatPath{6} = 'masters.last_change.nsec';
info.MatPath{7} = 'masters.last_change_local';
info.MatPath{8} = 'masters.last_change_local.sec';
info.MatPath{9} = 'masters.last_change_local.nsec';
info.MatPath{10} = 'masters.online';
info.MatPath{11} = 'masters.discoverer_name';
info.MatPath{12} = 'masters.monitoruri';
