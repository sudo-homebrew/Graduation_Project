function [data, info] = speechRecognitionResponse
%SpeechRecognition gives an empty data for speech_recognition_msgs/SpeechRecognitionResponse

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'speech_recognition_msgs/SpeechRecognitionResponse';
[data.Result, info.Result] = ros.internal.ros.messages.speech_recognition_msgs.speechRecognitionCandidates;
info.Result.MLdataType = 'struct';
info.MessageType = 'speech_recognition_msgs/SpeechRecognitionResponse';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,3);
info.MatPath{1} = 'result';
info.MatPath{2} = 'result.transcript';
info.MatPath{3} = 'result.confidence';
