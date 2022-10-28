function [data, info] = sendExpClusterRequest
%SendExpCluster gives an empty data for adhoc_communication/SendExpClusterRequest

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'adhoc_communication/SendExpClusterRequest';
[data.Topic, info.Topic] = ros.internal.ros.messages.ros.char('string',0);
[data.DstRobot, info.DstRobot] = ros.internal.ros.messages.ros.char('string',0);
[data.Cluster, info.Cluster] = ros.internal.ros.messages.adhoc_communication.expCluster;
info.Cluster.MLdataType = 'struct';
info.MessageType = 'adhoc_communication/SendExpClusterRequest';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,7);
info.MatPath{1} = 'topic';
info.MatPath{2} = 'dst_robot';
info.MatPath{3} = 'cluster';
info.MatPath{4} = 'cluster.ids_contained';
info.MatPath{5} = 'cluster.ids_contained.id';
info.MatPath{6} = 'cluster.ids_contained.detected_by_robot_str';
info.MatPath{7} = 'cluster.bid';
