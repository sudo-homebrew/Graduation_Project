function [data, info] = setWaveformRequest
%SetWaveform gives an empty data for ethercat_trigger_controllers/SetWaveformRequest

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'ethercat_trigger_controllers/SetWaveformRequest';
[data.RepRate, info.RepRate] = ros.internal.ros.messages.ros.default_type('double',1);
[data.Phase, info.Phase] = ros.internal.ros.messages.ros.default_type('double',1);
[data.DutyCycle, info.DutyCycle] = ros.internal.ros.messages.ros.default_type('double',1);
[data.Running, info.Running] = ros.internal.ros.messages.ros.default_type('int32',1);
[data.ActiveLow, info.ActiveLow] = ros.internal.ros.messages.ros.default_type('int32',1);
[data.Pulsed, info.Pulsed] = ros.internal.ros.messages.ros.default_type('int32',1);
info.MessageType = 'ethercat_trigger_controllers/SetWaveformRequest';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,6);
info.MatPath{1} = 'rep_rate';
info.MatPath{2} = 'phase';
info.MatPath{3} = 'duty_cycle';
info.MatPath{4} = 'running';
info.MatPath{5} = 'active_low';
info.MatPath{6} = 'pulsed';
