function [data, info] = audioData
%AudioData gives an empty data for audio_common_msgs/AudioData

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'audio_common_msgs/AudioData';
[data.Data, info.Data] = ros.internal.ros.messages.ros.default_type('uint8',NaN);
info.MessageType = 'audio_common_msgs/AudioData';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,1);
info.MatPath{1} = 'data';
