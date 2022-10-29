function [data, info] = freeCapabilityResponse
%FreeCapability gives an empty data for capabilities/FreeCapabilityResponse

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'capabilities/FreeCapabilityResponse';
info.MessageType = 'capabilities/FreeCapabilityResponse';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,0);
