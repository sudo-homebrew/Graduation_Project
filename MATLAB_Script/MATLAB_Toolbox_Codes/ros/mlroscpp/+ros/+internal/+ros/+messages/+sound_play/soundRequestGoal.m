function [data, info] = soundRequestGoal
%SoundRequestGoal gives an empty data for sound_play/SoundRequestGoal

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'sound_play/SoundRequestGoal';
[data.SoundRequest, info.SoundRequest] = ros.internal.ros.messages.sound_play.soundRequest;
info.SoundRequest.MLdataType = 'struct';
info.MessageType = 'sound_play/SoundRequestGoal';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,17);
info.MatPath{1} = 'sound_request';
info.MatPath{2} = 'sound_request.BACKINGUP';
info.MatPath{3} = 'sound_request.NEEDS_UNPLUGGING';
info.MatPath{4} = 'sound_request.NEEDS_PLUGGING';
info.MatPath{5} = 'sound_request.NEEDS_UNPLUGGING_BADLY';
info.MatPath{6} = 'sound_request.NEEDS_PLUGGING_BADLY';
info.MatPath{7} = 'sound_request.ALL';
info.MatPath{8} = 'sound_request.PLAY_FILE';
info.MatPath{9} = 'sound_request.SAY';
info.MatPath{10} = 'sound_request.sound';
info.MatPath{11} = 'sound_request.PLAY_STOP';
info.MatPath{12} = 'sound_request.PLAY_ONCE';
info.MatPath{13} = 'sound_request.PLAY_START';
info.MatPath{14} = 'sound_request.command';
info.MatPath{15} = 'sound_request.volume';
info.MatPath{16} = 'sound_request.arg';
info.MatPath{17} = 'sound_request.arg2';
