function [data, info] = grammar
%Grammar gives an empty data for speech_recognition_msgs/Grammar

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'speech_recognition_msgs/Grammar';
[data.Name, info.Name] = ros.internal.ros.messages.ros.char('string',0);
[data.Rules, info.Rules] = ros.internal.ros.messages.speech_recognition_msgs.phraseRule;
info.Rules.MLdataType = 'struct';
info.Rules.MaxLen = NaN;
info.Rules.MinLen = 0;
data.Rules = data.Rules([],1);
[data.Categories, info.Categories] = ros.internal.ros.messages.ros.char('string',NaN);
[data.Vocabularies, info.Vocabularies] = ros.internal.ros.messages.speech_recognition_msgs.vocabulary;
info.Vocabularies.MLdataType = 'struct';
info.Vocabularies.MaxLen = NaN;
info.Vocabularies.MinLen = 0;
data.Vocabularies = data.Vocabularies([],1);
info.MessageType = 'speech_recognition_msgs/Grammar';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,9);
info.MatPath{1} = 'name';
info.MatPath{2} = 'rules';
info.MatPath{3} = 'rules.symbol';
info.MatPath{4} = 'rules.definition';
info.MatPath{5} = 'categories';
info.MatPath{6} = 'vocabularies';
info.MatPath{7} = 'vocabularies.name';
info.MatPath{8} = 'vocabularies.words';
info.MatPath{9} = 'vocabularies.phonemes';
