function [data, info] = runningCapability
%RunningCapability gives an empty data for capabilities/RunningCapability

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'capabilities/RunningCapability';
[data.Capability, info.Capability] = ros.internal.ros.messages.capabilities.capability;
info.Capability.MLdataType = 'struct';
[data.DependentCapabilities, info.DependentCapabilities] = ros.internal.ros.messages.capabilities.capability;
info.DependentCapabilities.MLdataType = 'struct';
info.DependentCapabilities.MaxLen = NaN;
info.DependentCapabilities.MinLen = 0;
data.DependentCapabilities = data.DependentCapabilities([],1);
[data.StartedBy, info.StartedBy] = ros.internal.ros.messages.ros.char('string',0);
[data.Pid, info.Pid] = ros.internal.ros.messages.ros.default_type('int32',1);
info.MessageType = 'capabilities/RunningCapability';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,8);
info.MatPath{1} = 'capability';
info.MatPath{2} = 'capability.capability';
info.MatPath{3} = 'capability.provider';
info.MatPath{4} = 'dependent_capabilities';
info.MatPath{5} = 'dependent_capabilities.capability';
info.MatPath{6} = 'dependent_capabilities.provider';
info.MatPath{7} = 'started_by';
info.MatPath{8} = 'pid';
