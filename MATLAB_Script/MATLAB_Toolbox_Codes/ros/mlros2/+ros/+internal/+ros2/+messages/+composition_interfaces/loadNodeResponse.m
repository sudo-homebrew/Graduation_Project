function [data, info] = loadNodeResponse
%LoadNode gives an empty data for composition_interfaces/LoadNodeResponse

% Copyright 2019-2021 The MathWorks, Inc.
data = struct();
data.MessageType = 'composition_interfaces/LoadNodeResponse';
[data.success, info.success] = ros.internal.ros2.messages.ros2.default_type('logical',1,0);
[data.error_message, info.error_message] = ros.internal.ros2.messages.ros2.char('string',1,NaN,0);
[data.full_node_name, info.full_node_name] = ros.internal.ros2.messages.ros2.char('string',1,NaN,0);
[data.unique_id, info.unique_id] = ros.internal.ros2.messages.ros2.default_type('uint64',1,0);
info.MessageType = 'composition_interfaces/LoadNodeResponse';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,4);
info.MatPath{1} = 'success';
info.MatPath{2} = 'error_message';
info.MatPath{3} = 'full_node_name';
info.MatPath{4} = 'unique_id';
