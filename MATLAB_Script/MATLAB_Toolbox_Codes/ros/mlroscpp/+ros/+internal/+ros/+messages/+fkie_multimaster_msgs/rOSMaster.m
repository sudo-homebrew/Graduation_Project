function [data, info] = rOSMaster
%ROSMaster gives an empty data for fkie_multimaster_msgs/ROSMaster

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'fkie_multimaster_msgs/ROSMaster';
[data.Name, info.Name] = ros.internal.ros.messages.ros.char('string',0);
[data.Uri, info.Uri] = ros.internal.ros.messages.ros.char('string',0);
[data.LastChange, info.LastChange] = ros.internal.ros.messages.ros.time;
info.LastChange.MLdataType = 'struct';
[data.LastChangeLocal, info.LastChangeLocal] = ros.internal.ros.messages.ros.time;
info.LastChangeLocal.MLdataType = 'struct';
[data.Online, info.Online] = ros.internal.ros.messages.ros.default_type('logical',1);
[data.DiscovererName, info.DiscovererName] = ros.internal.ros.messages.ros.char('string',0);
[data.Monitoruri, info.Monitoruri] = ros.internal.ros.messages.ros.char('string',0);
info.MessageType = 'fkie_multimaster_msgs/ROSMaster';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,11);
info.MatPath{1} = 'name';
info.MatPath{2} = 'uri';
info.MatPath{3} = 'last_change';
info.MatPath{4} = 'last_change.sec';
info.MatPath{5} = 'last_change.nsec';
info.MatPath{6} = 'last_change_local';
info.MatPath{7} = 'last_change_local.sec';
info.MatPath{8} = 'last_change_local.nsec';
info.MatPath{9} = 'online';
info.MatPath{10} = 'discoverer_name';
info.MatPath{11} = 'monitoruri';
