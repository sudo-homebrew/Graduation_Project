function [data, info] = getCapabilitySpecResponse
%GetCapabilitySpec gives an empty data for capabilities/GetCapabilitySpecResponse

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'capabilities/GetCapabilitySpecResponse';
[data.CapabilitySpec, info.CapabilitySpec] = ros.internal.ros.messages.capabilities.capabilitySpec;
info.CapabilitySpec.MLdataType = 'struct';
info.MessageType = 'capabilities/GetCapabilitySpecResponse';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,5);
info.MatPath{1} = 'capability_spec';
info.MatPath{2} = 'capability_spec.package';
info.MatPath{3} = 'capability_spec.type';
info.MatPath{4} = 'capability_spec.content';
info.MatPath{5} = 'capability_spec.default_provider';
