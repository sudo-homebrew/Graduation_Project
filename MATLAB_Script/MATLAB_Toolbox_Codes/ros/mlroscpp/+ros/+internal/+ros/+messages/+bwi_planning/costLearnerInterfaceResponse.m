function [data, info] = costLearnerInterfaceResponse
%CostLearnerInterface gives an empty data for bwi_planning/CostLearnerInterfaceResponse

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'bwi_planning/CostLearnerInterfaceResponse';
[data.Success, info.Success] = ros.internal.ros.messages.ros.default_type('logical',1);
[data.Status, info.Status] = ros.internal.ros.messages.ros.char('string',0);
info.MessageType = 'bwi_planning/CostLearnerInterfaceResponse';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,2);
info.MatPath{1} = 'success';
info.MatPath{2} = 'status';
