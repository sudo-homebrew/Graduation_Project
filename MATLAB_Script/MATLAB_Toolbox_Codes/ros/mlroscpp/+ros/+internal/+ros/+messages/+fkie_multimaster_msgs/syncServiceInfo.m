function [data, info] = syncServiceInfo
%SyncServiceInfo gives an empty data for fkie_multimaster_msgs/SyncServiceInfo

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'fkie_multimaster_msgs/SyncServiceInfo';
[data.Service, info.Service] = ros.internal.ros.messages.ros.char('string',0);
[data.Serviceuri, info.Serviceuri] = ros.internal.ros.messages.ros.char('string',0);
[data.Node, info.Node] = ros.internal.ros.messages.ros.char('string',0);
[data.Nodeuri, info.Nodeuri] = ros.internal.ros.messages.ros.char('string',0);
info.MessageType = 'fkie_multimaster_msgs/SyncServiceInfo';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,4);
info.MatPath{1} = 'service';
info.MatPath{2} = 'serviceuri';
info.MatPath{3} = 'node';
info.MatPath{4} = 'nodeuri';
