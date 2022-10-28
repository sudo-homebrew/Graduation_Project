function [data, info] = plannerInterfaceResponse
%PlannerInterface gives an empty data for bwi_planning_common/PlannerInterfaceResponse

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'bwi_planning_common/PlannerInterfaceResponse';
[data.Success, info.Success] = ros.internal.ros.messages.ros.default_type('logical',1);
[data.Status, info.Status] = ros.internal.ros.messages.ros.char('string',0);
[data.Observations, info.Observations] = ros.internal.ros.messages.bwi_planning_common.plannerAtom;
info.Observations.MLdataType = 'struct';
info.Observations.MaxLen = NaN;
info.Observations.MinLen = 0;
data.Observations = data.Observations([],1);
info.MessageType = 'bwi_planning_common/PlannerInterfaceResponse';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,5);
info.MatPath{1} = 'success';
info.MatPath{2} = 'status';
info.MatPath{3} = 'observations';
info.MatPath{4} = 'observations.name';
info.MatPath{5} = 'observations.value';
