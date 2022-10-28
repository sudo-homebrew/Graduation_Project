function [data, info] = saveClassifierRequest
%SaveClassifier gives an empty data for ml_classifiers/SaveClassifierRequest

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'ml_classifiers/SaveClassifierRequest';
[data.Identifier, info.Identifier] = ros.internal.ros.messages.ros.char('string',0);
[data.Filename, info.Filename] = ros.internal.ros.messages.ros.char('string',0);
info.MessageType = 'ml_classifiers/SaveClassifierRequest';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,2);
info.MatPath{1} = 'identifier';
info.MatPath{2} = 'filename';
