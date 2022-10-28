function [data, info] = unloadNodeRequest
%UnloadNode gives an empty data for composition_interfaces/UnloadNodeRequest

% Copyright 2019-2021 The MathWorks, Inc.
data = struct();
data.MessageType = 'composition_interfaces/UnloadNodeRequest';
[data.unique_id, info.unique_id] = ros.internal.ros2.messages.ros2.default_type('uint64',1,0);
info.MessageType = 'composition_interfaces/UnloadNodeRequest';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,1);
info.MatPath{1} = 'unique_id';
