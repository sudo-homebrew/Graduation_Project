function [data, info] = setWaveformResponse
%SetWaveform gives an empty data for ethercat_trigger_controllers/SetWaveformResponse

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'ethercat_trigger_controllers/SetWaveformResponse';
info.MessageType = 'ethercat_trigger_controllers/SetWaveformResponse';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,0);
