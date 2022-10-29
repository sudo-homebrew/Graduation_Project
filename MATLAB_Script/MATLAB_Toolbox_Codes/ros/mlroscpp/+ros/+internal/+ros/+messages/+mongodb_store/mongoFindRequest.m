function [data, info] = mongoFindRequest
%MongoFind gives an empty data for mongodb_store/MongoFindRequest

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'mongodb_store/MongoFindRequest';
[data.Db, info.Db] = ros.internal.ros.messages.ros.char('string',0);
[data.Collection, info.Collection] = ros.internal.ros.messages.ros.char('string',0);
[data.Query, info.Query] = ros.internal.ros.messages.ros.char('string',0);
info.MessageType = 'mongodb_store/MongoFindRequest';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,3);
info.MatPath{1} = 'db';
info.MatPath{2} = 'collection';
info.MatPath{3} = 'query';
