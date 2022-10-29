function [data, info] = doorHandlerInterfaceRequest
%DoorHandlerInterface gives an empty data for segbot_simulation_apps/DoorHandlerInterfaceRequest

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'segbot_simulation_apps/DoorHandlerInterfaceRequest';
[data.Door, info.Door] = ros.internal.ros.messages.ros.char('string',0);
[data.Open, info.Open] = ros.internal.ros.messages.ros.default_type('logical',1);
[data.AllDoors, info.AllDoors] = ros.internal.ros.messages.ros.default_type('logical',1);
info.MessageType = 'segbot_simulation_apps/DoorHandlerInterfaceRequest';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,3);
info.MatPath{1} = 'door';
info.MatPath{2} = 'open';
info.MatPath{3} = 'all_doors';
