function [data, info] = implementation
%Implementation gives an empty data for concert_msgs/Implementation

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'concert_msgs/Implementation';
[data.Name, info.Name] = ros.internal.ros.messages.ros.char('string',0);
[data.LinkGraph, info.LinkGraph] = ros.internal.ros.messages.concert_msgs.linkGraph;
info.LinkGraph.MLdataType = 'struct';
[data.DotGraph, info.DotGraph] = ros.internal.ros.messages.ros.char('string',0);
info.MessageType = 'concert_msgs/Implementation';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,21);
info.MatPath{1} = 'name';
info.MatPath{2} = 'link_graph';
info.MatPath{3} = 'link_graph.nodes';
info.MatPath{4} = 'link_graph.nodes.id';
info.MatPath{5} = 'link_graph.nodes.tuple';
info.MatPath{6} = 'link_graph.nodes.UNLIMITED_RESOURCE';
info.MatPath{7} = 'link_graph.nodes.min';
info.MatPath{8} = 'link_graph.nodes.max';
info.MatPath{9} = 'link_graph.nodes.force_name_matching';
info.MatPath{10} = 'link_graph.topics';
info.MatPath{11} = 'link_graph.topics.id';
info.MatPath{12} = 'link_graph.topics.type';
info.MatPath{13} = 'link_graph.actions';
info.MatPath{14} = 'link_graph.actions.id';
info.MatPath{15} = 'link_graph.actions.type';
info.MatPath{16} = 'link_graph.edges';
info.MatPath{17} = 'link_graph.edges.start';
info.MatPath{18} = 'link_graph.edges.finish';
info.MatPath{19} = 'link_graph.edges.remap_from';
info.MatPath{20} = 'link_graph.edges.remap_to';
info.MatPath{21} = 'dot_graph';
