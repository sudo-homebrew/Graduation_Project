function [data, info] = useCapabilityRequest
%UseCapability gives an empty data for capabilities/UseCapabilityRequest

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'capabilities/UseCapabilityRequest';
[data.Capability, info.Capability] = ros.internal.ros.messages.ros.char('string',0);
[data.PreferredProvider, info.PreferredProvider] = ros.internal.ros.messages.ros.char('string',0);
[data.BondId, info.BondId] = ros.internal.ros.messages.ros.char('string',0);
info.MessageType = 'capabilities/UseCapabilityRequest';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,3);
info.MatPath{1} = 'capability';
info.MatPath{2} = 'preferred_provider';
info.MatPath{3} = 'bond_id';
