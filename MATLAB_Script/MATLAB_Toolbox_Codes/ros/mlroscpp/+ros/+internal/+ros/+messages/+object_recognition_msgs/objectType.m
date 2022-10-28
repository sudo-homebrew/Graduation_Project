function [data, info] = objectType
%ObjectType gives an empty data for object_recognition_msgs/ObjectType

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'object_recognition_msgs/ObjectType';
[data.Key, info.Key] = ros.internal.ros.messages.ros.char('string',0);
[data.Db, info.Db] = ros.internal.ros.messages.ros.char('string',0);
info.MessageType = 'object_recognition_msgs/ObjectType';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,2);
info.MatPath{1} = 'key';
info.MatPath{2} = 'db';
