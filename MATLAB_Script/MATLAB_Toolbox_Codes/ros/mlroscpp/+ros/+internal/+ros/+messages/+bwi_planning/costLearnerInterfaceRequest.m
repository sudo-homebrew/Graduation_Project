function [data, info] = costLearnerInterfaceRequest
%CostLearnerInterface gives an empty data for bwi_planning/CostLearnerInterfaceRequest

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'bwi_planning/CostLearnerInterfaceRequest';
[data.Location, info.Location] = ros.internal.ros.messages.ros.char('string',0);
[data.DoorFrom, info.DoorFrom] = ros.internal.ros.messages.ros.char('string',0);
[data.DoorTo, info.DoorTo] = ros.internal.ros.messages.ros.char('string',0);
[data.Cost, info.Cost] = ros.internal.ros.messages.ros.default_type('single',1);
info.MessageType = 'bwi_planning/CostLearnerInterfaceRequest';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,4);
info.MatPath{1} = 'location';
info.MatPath{2} = 'door_from';
info.MatPath{3} = 'door_to';
info.MatPath{4} = 'cost';
