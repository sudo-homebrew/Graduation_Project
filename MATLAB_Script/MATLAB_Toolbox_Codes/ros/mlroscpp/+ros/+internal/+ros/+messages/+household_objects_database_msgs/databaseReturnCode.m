function [data, info] = databaseReturnCode
%DatabaseReturnCode gives an empty data for household_objects_database_msgs/DatabaseReturnCode

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'household_objects_database_msgs/DatabaseReturnCode';
[data.UNKNOWNERROR, info.UNKNOWNERROR] = ros.internal.ros.messages.ros.default_type('int32',1, 1);
[data.DATABASENOTCONNECTED, info.DATABASENOTCONNECTED] = ros.internal.ros.messages.ros.default_type('int32',1, 2);
[data.DATABASEQUERYERROR, info.DATABASEQUERYERROR] = ros.internal.ros.messages.ros.default_type('int32',1, 3);
[data.SUCCESS, info.SUCCESS] = ros.internal.ros.messages.ros.default_type('int32',1, -1);
[data.Code, info.Code] = ros.internal.ros.messages.ros.default_type('int32',1);
info.MessageType = 'household_objects_database_msgs/DatabaseReturnCode';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,5);
info.MatPath{1} = 'UNKNOWN_ERROR';
info.MatPath{2} = 'DATABASE_NOT_CONNECTED';
info.MatPath{3} = 'DATABASE_QUERY_ERROR';
info.MatPath{4} = 'SUCCESS';
info.MatPath{5} = 'code';
