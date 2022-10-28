function [data, info] = soundRequestFeedback
%SoundRequestFeedback gives an empty data for sound_play/SoundRequestFeedback

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'sound_play/SoundRequestFeedback';
[data.Playing, info.Playing] = ros.internal.ros.messages.ros.default_type('logical',1);
[data.Stamp, info.Stamp] = ros.internal.ros.messages.ros.time;
info.Stamp.MLdataType = 'struct';
info.MessageType = 'sound_play/SoundRequestFeedback';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,4);
info.MatPath{1} = 'playing';
info.MatPath{2} = 'stamp';
info.MatPath{3} = 'stamp.sec';
info.MatPath{4} = 'stamp.nsec';
