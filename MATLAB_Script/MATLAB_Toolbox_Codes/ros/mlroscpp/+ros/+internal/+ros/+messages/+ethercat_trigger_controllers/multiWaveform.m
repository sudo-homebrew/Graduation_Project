function [data, info] = multiWaveform
%MultiWaveform gives an empty data for ethercat_trigger_controllers/MultiWaveform

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'ethercat_trigger_controllers/MultiWaveform';
[data.Period, info.Period] = ros.internal.ros.messages.ros.default_type('double',1);
[data.ZeroOffset, info.ZeroOffset] = ros.internal.ros.messages.ros.default_type('double',1);
[data.Transitions, info.Transitions] = ros.internal.ros.messages.ethercat_trigger_controllers.multiWaveformTransition;
info.Transitions.MLdataType = 'struct';
info.Transitions.MaxLen = NaN;
info.Transitions.MinLen = 0;
data.Transitions = data.Transitions([],1);
info.MessageType = 'ethercat_trigger_controllers/MultiWaveform';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,6);
info.MatPath{1} = 'period';
info.MatPath{2} = 'zero_offset';
info.MatPath{3} = 'transitions';
info.MatPath{4} = 'transitions.time';
info.MatPath{5} = 'transitions.value';
info.MatPath{6} = 'transitions.topic';
