function [data, info] = stopCapabilityRequest
%StopCapability gives an empty data for capabilities/StopCapabilityRequest

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'capabilities/StopCapabilityRequest';
[data.Capability, info.Capability] = ros.internal.ros.messages.ros.char('string',0);
info.MessageType = 'capabilities/StopCapabilityRequest';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,1);
info.MatPath{1} = 'capability';
