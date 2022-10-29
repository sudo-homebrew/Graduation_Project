function [data, info] = triggerRequest
%Trigger gives an empty data for std_srvs/TriggerRequest

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'std_srvs/TriggerRequest';
info.MessageType = 'std_srvs/TriggerRequest';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,0);
