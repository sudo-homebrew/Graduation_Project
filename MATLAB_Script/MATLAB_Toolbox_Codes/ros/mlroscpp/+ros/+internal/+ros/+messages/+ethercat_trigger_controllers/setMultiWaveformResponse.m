function [data, info] = setMultiWaveformResponse
%SetMultiWaveform gives an empty data for ethercat_trigger_controllers/SetMultiWaveformResponse

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'ethercat_trigger_controllers/SetMultiWaveformResponse';
[data.Success, info.Success] = ros.internal.ros.messages.ros.default_type('logical',1);
[data.StatusMessage, info.StatusMessage] = ros.internal.ros.messages.ros.char('string',0);
info.MessageType = 'ethercat_trigger_controllers/SetMultiWaveformResponse';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,2);
info.MatPath{1} = 'success';
info.MatPath{2} = 'status_message';
