function [data, info] = expCluster
%ExpCluster gives an empty data for adhoc_communication/ExpCluster

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'adhoc_communication/ExpCluster';
[data.IdsContained, info.IdsContained] = ros.internal.ros.messages.adhoc_communication.expClusterElement;
info.IdsContained.MLdataType = 'struct';
info.IdsContained.MaxLen = NaN;
info.IdsContained.MinLen = 0;
data.IdsContained = data.IdsContained([],1);
[data.Bid, info.Bid] = ros.internal.ros.messages.ros.default_type('double',1);
info.MessageType = 'adhoc_communication/ExpCluster';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,4);
info.MatPath{1} = 'ids_contained';
info.MatPath{2} = 'ids_contained.id';
info.MatPath{3} = 'ids_contained.detected_by_robot_str';
info.MatPath{4} = 'bid';
