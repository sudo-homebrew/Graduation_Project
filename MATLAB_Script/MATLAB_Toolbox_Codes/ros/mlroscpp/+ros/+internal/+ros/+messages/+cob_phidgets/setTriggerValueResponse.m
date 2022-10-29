function [data, info] = setTriggerValueResponse
%SetTriggerValue gives an empty data for cob_phidgets/SetTriggerValueResponse

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'cob_phidgets/SetTriggerValueResponse';
info.MessageType = 'cob_phidgets/SetTriggerValueResponse';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,0);
