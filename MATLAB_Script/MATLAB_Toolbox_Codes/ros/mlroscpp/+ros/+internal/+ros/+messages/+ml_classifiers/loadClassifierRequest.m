function [data, info] = loadClassifierRequest
%LoadClassifier gives an empty data for ml_classifiers/LoadClassifierRequest

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'ml_classifiers/LoadClassifierRequest';
[data.Identifier, info.Identifier] = ros.internal.ros.messages.ros.char('string',0);
[data.ClassType, info.ClassType] = ros.internal.ros.messages.ros.char('string',0);
[data.Filename, info.Filename] = ros.internal.ros.messages.ros.char('string',0);
info.MessageType = 'ml_classifiers/LoadClassifierRequest';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,3);
info.MatPath{1} = 'identifier';
info.MatPath{2} = 'class_type';
info.MatPath{3} = 'filename';
