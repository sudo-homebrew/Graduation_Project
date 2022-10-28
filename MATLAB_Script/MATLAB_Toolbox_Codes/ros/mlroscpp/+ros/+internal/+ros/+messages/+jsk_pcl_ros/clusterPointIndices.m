function [data, info] = clusterPointIndices
%ClusterPointIndices gives an empty data for jsk_pcl_ros/ClusterPointIndices

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'jsk_pcl_ros/ClusterPointIndices';
[data.Header, info.Header] = ros.internal.ros.messages.std_msgs.header;
info.Header.MLdataType = 'struct';
[data.ClusterIndices, info.ClusterIndices] = ros.internal.ros.messages.pcl_msgs.pointIndices;
info.ClusterIndices.MLdataType = 'struct';
info.ClusterIndices.MaxLen = NaN;
info.ClusterIndices.MinLen = 0;
data.ClusterIndices = data.ClusterIndices([],1);
info.MessageType = 'jsk_pcl_ros/ClusterPointIndices';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,14);
info.MatPath{1} = 'header';
info.MatPath{2} = 'header.seq';
info.MatPath{3} = 'header.stamp';
info.MatPath{4} = 'header.stamp.sec';
info.MatPath{5} = 'header.stamp.nsec';
info.MatPath{6} = 'header.frame_id';
info.MatPath{7} = 'cluster_indices';
info.MatPath{8} = 'cluster_indices.header';
info.MatPath{9} = 'cluster_indices.header.seq';
info.MatPath{10} = 'cluster_indices.header.stamp';
info.MatPath{11} = 'cluster_indices.header.stamp.sec';
info.MatPath{12} = 'cluster_indices.header.stamp.nsec';
info.MatPath{13} = 'cluster_indices.header.frame_id';
info.MatPath{14} = 'cluster_indices.indices';
