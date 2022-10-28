function [data, info] = startCapabilityRequest
%StartCapability gives an empty data for capabilities/StartCapabilityRequest

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'capabilities/StartCapabilityRequest';
[data.Capability, info.Capability] = ros.internal.ros.messages.ros.char('string',0);
[data.PreferredProvider, info.PreferredProvider] = ros.internal.ros.messages.ros.char('string',0);
info.MessageType = 'capabilities/StartCapabilityRequest';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,2);
info.MatPath{1} = 'capability';
info.MatPath{2} = 'preferred_provider';
