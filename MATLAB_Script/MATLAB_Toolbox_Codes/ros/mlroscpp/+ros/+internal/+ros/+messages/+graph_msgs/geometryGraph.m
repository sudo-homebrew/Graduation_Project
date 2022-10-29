function [data, info] = geometryGraph
%GeometryGraph gives an empty data for graph_msgs/GeometryGraph

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'graph_msgs/GeometryGraph';
[data.Header, info.Header] = ros.internal.ros.messages.std_msgs.header;
info.Header.MLdataType = 'struct';
[data.Nodes, info.Nodes] = ros.internal.ros.messages.geometry_msgs.point;
info.Nodes.MLdataType = 'struct';
info.Nodes.MaxLen = NaN;
info.Nodes.MinLen = 0;
data.Nodes = data.Nodes([],1);
[data.Edges, info.Edges] = ros.internal.ros.messages.graph_msgs.edges;
info.Edges.MLdataType = 'struct';
info.Edges.MaxLen = NaN;
info.Edges.MinLen = 0;
data.Edges = data.Edges([],1);
info.MessageType = 'graph_msgs/GeometryGraph';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,13);
info.MatPath{1} = 'header';
info.MatPath{2} = 'header.seq';
info.MatPath{3} = 'header.stamp';
info.MatPath{4} = 'header.stamp.sec';
info.MatPath{5} = 'header.stamp.nsec';
info.MatPath{6} = 'header.frame_id';
info.MatPath{7} = 'nodes';
info.MatPath{8} = 'nodes.x';
info.MatPath{9} = 'nodes.y';
info.MatPath{10} = 'nodes.z';
info.MatPath{11} = 'edges';
info.MatPath{12} = 'edges.node_ids';
info.MatPath{13} = 'edges.weights';
