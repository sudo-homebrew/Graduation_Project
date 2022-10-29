function [data, info] = doorHandlerInterfaceResponse
%DoorHandlerInterface gives an empty data for segbot_simulation_apps/DoorHandlerInterfaceResponse

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'segbot_simulation_apps/DoorHandlerInterfaceResponse';
[data.Success, info.Success] = ros.internal.ros.messages.ros.default_type('logical',1);
[data.Status, info.Status] = ros.internal.ros.messages.ros.char('string',0);
info.MessageType = 'segbot_simulation_apps/DoorHandlerInterfaceResponse';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,2);
info.MatPath{1} = 'success';
info.MatPath{2} = 'status';
