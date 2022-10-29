function [data, info] = getRunningCapabilitiesResponse
%GetRunningCapabilities gives an empty data for capabilities/GetRunningCapabilitiesResponse

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'capabilities/GetRunningCapabilitiesResponse';
[data.RunningCapabilities, info.RunningCapabilities] = ros.internal.ros.messages.capabilities.runningCapability;
info.RunningCapabilities.MLdataType = 'struct';
info.RunningCapabilities.MaxLen = NaN;
info.RunningCapabilities.MinLen = 0;
data.RunningCapabilities = data.RunningCapabilities([],1);
info.MessageType = 'capabilities/GetRunningCapabilitiesResponse';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,9);
info.MatPath{1} = 'running_capabilities';
info.MatPath{2} = 'running_capabilities.capability';
info.MatPath{3} = 'running_capabilities.capability.capability';
info.MatPath{4} = 'running_capabilities.capability.provider';
info.MatPath{5} = 'running_capabilities.dependent_capabilities';
info.MatPath{6} = 'running_capabilities.dependent_capabilities.capability';
info.MatPath{7} = 'running_capabilities.dependent_capabilities.provider';
info.MatPath{8} = 'running_capabilities.started_by';
info.MatPath{9} = 'running_capabilities.pid';
