function [data, info] = phraseRule
%PhraseRule gives an empty data for speech_recognition_msgs/PhraseRule

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'speech_recognition_msgs/PhraseRule';
[data.Symbol, info.Symbol] = ros.internal.ros.messages.ros.char('string',0);
[data.Definition, info.Definition] = ros.internal.ros.messages.ros.char('string',NaN);
info.MessageType = 'speech_recognition_msgs/PhraseRule';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,2);
info.MatPath{1} = 'symbol';
info.MatPath{2} = 'definition';
