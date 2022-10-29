function [data, info] = audioInfo
%AudioInfo gives an empty data for audio_common_msgs/AudioInfo

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'audio_common_msgs/AudioInfo';
[data.Channels, info.Channels] = ros.internal.ros.messages.ros.default_type('uint8',1);
[data.SampleRate, info.SampleRate] = ros.internal.ros.messages.ros.default_type('uint32',1);
[data.SampleFormat, info.SampleFormat] = ros.internal.ros.messages.ros.char('string',0);
[data.Bitrate, info.Bitrate] = ros.internal.ros.messages.ros.default_type('uint32',1);
[data.CodingFormat, info.CodingFormat] = ros.internal.ros.messages.ros.char('string',0);
info.MessageType = 'audio_common_msgs/AudioInfo';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,5);
info.MatPath{1} = 'channels';
info.MatPath{2} = 'sample_rate';
info.MatPath{3} = 'sample_format';
info.MatPath{4} = 'bitrate';
info.MatPath{5} = 'coding_format';
