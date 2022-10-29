function [data, info] = createClassifierRequest
%CreateClassifier gives an empty data for ml_classifiers/CreateClassifierRequest

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'ml_classifiers/CreateClassifierRequest';
[data.Identifier, info.Identifier] = ros.internal.ros.messages.ros.char('string',0);
[data.ClassType, info.ClassType] = ros.internal.ros.messages.ros.char('string',0);
info.MessageType = 'ml_classifiers/CreateClassifierRequest';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,2);
info.MatPath{1} = 'identifier';
info.MatPath{2} = 'class_type';
