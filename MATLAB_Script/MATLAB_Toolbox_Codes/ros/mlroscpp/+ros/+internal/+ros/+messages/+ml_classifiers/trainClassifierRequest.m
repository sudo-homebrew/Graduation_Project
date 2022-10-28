function [data, info] = trainClassifierRequest
%TrainClassifier gives an empty data for ml_classifiers/TrainClassifierRequest

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'ml_classifiers/TrainClassifierRequest';
[data.Identifier, info.Identifier] = ros.internal.ros.messages.ros.char('string',0);
info.MessageType = 'ml_classifiers/TrainClassifierRequest';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,1);
info.MatPath{1} = 'identifier';
