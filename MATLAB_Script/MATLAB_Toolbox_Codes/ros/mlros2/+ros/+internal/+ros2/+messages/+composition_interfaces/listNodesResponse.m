function [data, info] = listNodesResponse
%ListNodes gives an empty data for composition_interfaces/ListNodesResponse

% Copyright 2019-2021 The MathWorks, Inc.
data = struct();
data.MessageType = 'composition_interfaces/ListNodesResponse';
[data.full_node_names, info.full_node_names] = ros.internal.ros2.messages.ros2.char('string',NaN,NaN,0);
[data.unique_ids, info.unique_ids] = ros.internal.ros2.messages.ros2.default_type('uint64',NaN,0);
info.MessageType = 'composition_interfaces/ListNodesResponse';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,2);
info.MatPath{1} = 'full_node_names';
info.MatPath{2} = 'unique_ids';
