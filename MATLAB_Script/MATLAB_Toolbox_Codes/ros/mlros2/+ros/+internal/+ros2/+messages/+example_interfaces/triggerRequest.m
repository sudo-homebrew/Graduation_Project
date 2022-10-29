function [data, info] = triggerRequest
%Trigger gives an empty data for example_interfaces/TriggerRequest

% Copyright 2019-2021 The MathWorks, Inc.
data = struct();
data.MessageType = 'example_interfaces/TriggerRequest';
info.MessageType = 'example_interfaces/TriggerRequest';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,0);
