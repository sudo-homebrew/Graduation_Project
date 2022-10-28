function [data, info] = speechRecognitionCandidates
%SpeechRecognitionCandidates gives an empty data for speech_recognition_msgs/SpeechRecognitionCandidates

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'speech_recognition_msgs/SpeechRecognitionCandidates';
[data.Transcript, info.Transcript] = ros.internal.ros.messages.ros.char('string',NaN);
[data.Confidence, info.Confidence] = ros.internal.ros.messages.ros.default_type('single',NaN);
info.MessageType = 'speech_recognition_msgs/SpeechRecognitionCandidates';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,2);
info.MatPath{1} = 'transcript';
info.MatPath{2} = 'confidence';
