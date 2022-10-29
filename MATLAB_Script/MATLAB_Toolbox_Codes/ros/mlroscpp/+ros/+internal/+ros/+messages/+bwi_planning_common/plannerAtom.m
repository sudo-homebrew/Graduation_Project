function [data, info] = plannerAtom
%PlannerAtom gives an empty data for bwi_planning_common/PlannerAtom

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'bwi_planning_common/PlannerAtom';
[data.Name, info.Name] = ros.internal.ros.messages.ros.char('string',0);
[data.Value, info.Value] = ros.internal.ros.messages.ros.char('string',NaN);
info.MessageType = 'bwi_planning_common/PlannerAtom';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,2);
info.MatPath{1} = 'name';
info.MatPath{2} = 'value';
