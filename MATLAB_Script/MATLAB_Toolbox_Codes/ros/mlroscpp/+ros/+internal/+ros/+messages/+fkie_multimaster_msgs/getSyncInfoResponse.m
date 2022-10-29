function [data, info] = getSyncInfoResponse
%GetSyncInfo gives an empty data for fkie_multimaster_msgs/GetSyncInfoResponse

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'fkie_multimaster_msgs/GetSyncInfoResponse';
[data.Hosts, info.Hosts] = ros.internal.ros.messages.fkie_multimaster_msgs.syncMasterInfo;
info.Hosts.MLdataType = 'struct';
info.Hosts.MaxLen = NaN;
info.Hosts.MinLen = 0;
data.Hosts = data.Hosts([],1);
info.MessageType = 'fkie_multimaster_msgs/GetSyncInfoResponse';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,16);
info.MatPath{1} = 'hosts';
info.MatPath{2} = 'hosts.masteruri';
info.MatPath{3} = 'hosts.nodes';
info.MatPath{4} = 'hosts.publisher';
info.MatPath{5} = 'hosts.publisher.topic';
info.MatPath{6} = 'hosts.publisher.node';
info.MatPath{7} = 'hosts.publisher.nodeuri';
info.MatPath{8} = 'hosts.subscriber';
info.MatPath{9} = 'hosts.subscriber.topic';
info.MatPath{10} = 'hosts.subscriber.node';
info.MatPath{11} = 'hosts.subscriber.nodeuri';
info.MatPath{12} = 'hosts.services';
info.MatPath{13} = 'hosts.services.service';
info.MatPath{14} = 'hosts.services.serviceuri';
info.MatPath{15} = 'hosts.services.node';
info.MatPath{16} = 'hosts.services.nodeuri';
