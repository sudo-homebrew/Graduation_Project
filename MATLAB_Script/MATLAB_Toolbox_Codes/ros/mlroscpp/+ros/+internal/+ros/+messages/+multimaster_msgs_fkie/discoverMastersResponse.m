function [data, info] = discoverMastersResponse
%DiscoverMasters gives an empty data for multimaster_msgs_fkie/DiscoverMastersResponse

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'multimaster_msgs_fkie/DiscoverMastersResponse';
[data.Masters, info.Masters] = ros.internal.ros.messages.multimaster_msgs_fkie.rOSMaster;
info.Masters.MLdataType = 'struct';
info.Masters.MaxLen = NaN;
info.Masters.MinLen = 0;
data.Masters = data.Masters([],1);
info.MessageType = 'multimaster_msgs_fkie/DiscoverMastersResponse';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,8);
info.MatPath{1} = 'masters';
info.MatPath{2} = 'masters.name';
info.MatPath{3} = 'masters.uri';
info.MatPath{4} = 'masters.timestamp';
info.MatPath{5} = 'masters.timestamp_local';
info.MatPath{6} = 'masters.online';
info.MatPath{7} = 'masters.discoverer_name';
info.MatPath{8} = 'masters.monitoruri';
