function [data, info] = softProcessorFirmwareReadRequest
%SoftProcessorFirmwareRead gives an empty data for ethercat_hardware/SoftProcessorFirmwareReadRequest

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'ethercat_hardware/SoftProcessorFirmwareReadRequest';
[data.ActuatorName, info.ActuatorName] = ros.internal.ros.messages.ros.char('string',0);
[data.ProcessorName, info.ProcessorName] = ros.internal.ros.messages.ros.char('string',0);
info.MessageType = 'ethercat_hardware/SoftProcessorFirmwareReadRequest';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,2);
info.MatPath{1} = 'actuator_name';
info.MatPath{2} = 'processor_name';
