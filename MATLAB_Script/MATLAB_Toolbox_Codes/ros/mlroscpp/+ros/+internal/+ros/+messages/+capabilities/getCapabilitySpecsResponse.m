function [data, info] = getCapabilitySpecsResponse
%GetCapabilitySpecs gives an empty data for capabilities/GetCapabilitySpecsResponse

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'capabilities/GetCapabilitySpecsResponse';
[data.CapabilitySpecs, info.CapabilitySpecs] = ros.internal.ros.messages.capabilities.capabilitySpec;
info.CapabilitySpecs.MLdataType = 'struct';
info.CapabilitySpecs.MaxLen = NaN;
info.CapabilitySpecs.MinLen = 0;
data.CapabilitySpecs = data.CapabilitySpecs([],1);
info.MessageType = 'capabilities/GetCapabilitySpecsResponse';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,5);
info.MatPath{1} = 'capability_specs';
info.MatPath{2} = 'capability_specs.package';
info.MatPath{3} = 'capability_specs.type';
info.MatPath{4} = 'capability_specs.content';
info.MatPath{5} = 'capability_specs.default_provider';
