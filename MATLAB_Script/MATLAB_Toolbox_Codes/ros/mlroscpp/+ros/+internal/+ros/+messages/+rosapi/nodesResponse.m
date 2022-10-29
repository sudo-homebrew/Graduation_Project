function [data, info] = nodesResponse
%Nodes gives an empty data for rosapi/NodesResponse

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'rosapi/NodesResponse';
[data.Nodes, info.Nodes] = ros.internal.ros.messages.ros.char('string',NaN);
info.MessageType = 'rosapi/NodesResponse';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,1);
info.MatPath{1} = 'nodes';
