function [data, info] = addClassDataResponse
%AddClassData gives an empty data for ml_classifiers/AddClassDataResponse

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'ml_classifiers/AddClassDataResponse';
[data.Success, info.Success] = ros.internal.ros.messages.ros.default_type('logical',1);
info.MessageType = 'ml_classifiers/AddClassDataResponse';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,1);
info.MatPath{1} = 'success';
