function [data, info] = setLightModeGoal
%SetLightModeGoal gives an empty data for cob_light/SetLightModeGoal

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'cob_light/SetLightModeGoal';
[data.Mode, info.Mode] = ros.internal.ros.messages.cob_light.lightMode;
info.Mode.MLdataType = 'struct';
info.MessageType = 'cob_light/SetLightModeGoal';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,19);
info.MatPath{1} = 'mode';
info.MatPath{2} = 'mode.mode';
info.MatPath{3} = 'mode.frequency';
info.MatPath{4} = 'mode.timeout';
info.MatPath{5} = 'mode.pulses';
info.MatPath{6} = 'mode.priority';
info.MatPath{7} = 'mode.colors';
info.MatPath{8} = 'mode.colors.r';
info.MatPath{9} = 'mode.colors.g';
info.MatPath{10} = 'mode.colors.b';
info.MatPath{11} = 'mode.colors.a';
info.MatPath{12} = 'mode.sequences';
info.MatPath{13} = 'mode.sequences.color';
info.MatPath{14} = 'mode.sequences.color.r';
info.MatPath{15} = 'mode.sequences.color.g';
info.MatPath{16} = 'mode.sequences.color.b';
info.MatPath{17} = 'mode.sequences.color.a';
info.MatPath{18} = 'mode.sequences.hold_time';
info.MatPath{19} = 'mode.sequences.cross_time';
