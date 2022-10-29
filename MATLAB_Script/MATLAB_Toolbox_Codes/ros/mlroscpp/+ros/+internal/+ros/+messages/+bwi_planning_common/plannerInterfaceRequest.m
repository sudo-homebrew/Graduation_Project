function [data, info] = plannerInterfaceRequest
%PlannerInterface gives an empty data for bwi_planning_common/PlannerInterfaceRequest

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'bwi_planning_common/PlannerInterfaceRequest';
[data.Command, info.Command] = ros.internal.ros.messages.bwi_planning_common.plannerAtom;
info.Command.MLdataType = 'struct';
info.MessageType = 'bwi_planning_common/PlannerInterfaceRequest';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,3);
info.MatPath{1} = 'command';
info.MatPath{2} = 'command.name';
info.MatPath{3} = 'command.value';
