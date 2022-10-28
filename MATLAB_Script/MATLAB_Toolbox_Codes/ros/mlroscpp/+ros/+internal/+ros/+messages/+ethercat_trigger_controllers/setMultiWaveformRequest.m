function [data, info] = setMultiWaveformRequest
%SetMultiWaveform gives an empty data for ethercat_trigger_controllers/SetMultiWaveformRequest

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'ethercat_trigger_controllers/SetMultiWaveformRequest';
[data.Waveform, info.Waveform] = ros.internal.ros.messages.ethercat_trigger_controllers.multiWaveform;
info.Waveform.MLdataType = 'struct';
info.MessageType = 'ethercat_trigger_controllers/SetMultiWaveformRequest';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,7);
info.MatPath{1} = 'waveform';
info.MatPath{2} = 'waveform.period';
info.MatPath{3} = 'waveform.zero_offset';
info.MatPath{4} = 'waveform.transitions';
info.MatPath{5} = 'waveform.transitions.time';
info.MatPath{6} = 'waveform.transitions.value';
info.MatPath{7} = 'waveform.transitions.topic';
