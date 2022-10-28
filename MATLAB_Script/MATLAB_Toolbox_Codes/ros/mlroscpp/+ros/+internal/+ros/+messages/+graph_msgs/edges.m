function [data, info] = edges
%Edges gives an empty data for graph_msgs/Edges

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'graph_msgs/Edges';
[data.NodeIds, info.NodeIds] = ros.internal.ros.messages.ros.default_type('uint32',NaN);
[data.Weights, info.Weights] = ros.internal.ros.messages.ros.default_type('double',NaN);
info.MessageType = 'graph_msgs/Edges';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,2);
info.MatPath{1} = 'node_ids';
info.MatPath{2} = 'weights';
