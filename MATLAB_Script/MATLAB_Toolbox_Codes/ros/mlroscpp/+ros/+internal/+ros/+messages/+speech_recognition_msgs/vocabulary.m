function [data, info] = vocabulary
%Vocabulary gives an empty data for speech_recognition_msgs/Vocabulary

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'speech_recognition_msgs/Vocabulary';
[data.Name, info.Name] = ros.internal.ros.messages.ros.char('string',0);
[data.Words, info.Words] = ros.internal.ros.messages.ros.char('string',NaN);
[data.Phonemes, info.Phonemes] = ros.internal.ros.messages.ros.char('string',NaN);
info.MessageType = 'speech_recognition_msgs/Vocabulary';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,3);
info.MatPath{1} = 'name';
info.MatPath{2} = 'words';
info.MatPath{3} = 'phonemes';
