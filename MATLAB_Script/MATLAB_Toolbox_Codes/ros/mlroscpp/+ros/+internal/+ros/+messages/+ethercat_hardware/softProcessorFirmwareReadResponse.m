function [data, info] = softProcessorFirmwareReadResponse
%SoftProcessorFirmwareRead gives an empty data for ethercat_hardware/SoftProcessorFirmwareReadResponse

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'ethercat_hardware/SoftProcessorFirmwareReadResponse';
[data.Success, info.Success] = ros.internal.ros.messages.ros.default_type('logical',1);
[data.ErrorMsg, info.ErrorMsg] = ros.internal.ros.messages.ros.char('string',0);
[data.Instructions, info.Instructions] = ros.internal.ros.messages.ros.default_type('uint32',NaN);
info.MessageType = 'ethercat_hardware/SoftProcessorFirmwareReadResponse';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,3);
info.MatPath{1} = 'success';
info.MatPath{2} = 'error_msg';
info.MatPath{3} = 'instructions';
