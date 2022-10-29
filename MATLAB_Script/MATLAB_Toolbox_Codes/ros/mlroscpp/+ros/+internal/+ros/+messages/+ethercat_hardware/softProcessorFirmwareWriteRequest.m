function [data, info] = softProcessorFirmwareWriteRequest
%SoftProcessorFirmwareWrite gives an empty data for ethercat_hardware/SoftProcessorFirmwareWriteRequest

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'ethercat_hardware/SoftProcessorFirmwareWriteRequest';
[data.ActuatorName, info.ActuatorName] = ros.internal.ros.messages.ros.char('string',0);
[data.ProcessorName, info.ProcessorName] = ros.internal.ros.messages.ros.char('string',0);
[data.Instructions, info.Instructions] = ros.internal.ros.messages.ros.default_type('uint32',NaN);
info.MessageType = 'ethercat_hardware/SoftProcessorFirmwareWriteRequest';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,3);
info.MatPath{1} = 'actuator_name';
info.MatPath{2} = 'processor_name';
info.MatPath{3} = 'instructions';
