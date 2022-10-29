function [data, info] = classifyDataResponse
%ClassifyData gives an empty data for ml_classifiers/ClassifyDataResponse

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'ml_classifiers/ClassifyDataResponse';
[data.Classifications, info.Classifications] = ros.internal.ros.messages.ros.char('string',NaN);
info.MessageType = 'ml_classifiers/ClassifyDataResponse';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,1);
info.MatPath{1} = 'classifications';
