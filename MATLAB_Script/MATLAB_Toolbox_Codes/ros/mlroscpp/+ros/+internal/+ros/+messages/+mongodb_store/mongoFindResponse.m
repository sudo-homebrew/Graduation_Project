function [data, info] = mongoFindResponse
%MongoFind gives an empty data for mongodb_store/MongoFindResponse

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'mongodb_store/MongoFindResponse';
[data.Result, info.Result] = ros.internal.ros.messages.ros.char('string',0);
info.MessageType = 'mongodb_store/MongoFindResponse';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,1);
info.MatPath{1} = 'result';