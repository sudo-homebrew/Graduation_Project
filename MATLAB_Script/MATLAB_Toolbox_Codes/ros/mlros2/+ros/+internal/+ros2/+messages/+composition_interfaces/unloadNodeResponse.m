function [data, info] = unloadNodeResponse
%UnloadNode gives an empty data for composition_interfaces/UnloadNodeResponse

% Copyright 2019-2021 The MathWorks, Inc.
data = struct();
data.MessageType = 'composition_interfaces/UnloadNodeResponse';
[data.success, info.success] = ros.internal.ros2.messages.ros2.default_type('logical',1,0);
[data.error_message, info.error_message] = ros.internal.ros2.messages.ros2.char('string',1,NaN,0);
info.MessageType = 'composition_interfaces/UnloadNodeResponse';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,2);
info.MatPath{1} = 'success';
info.MatPath{2} = 'error_message';
