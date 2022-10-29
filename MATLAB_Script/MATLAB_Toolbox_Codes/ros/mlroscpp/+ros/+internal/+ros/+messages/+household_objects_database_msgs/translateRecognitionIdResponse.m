function [data, info] = translateRecognitionIdResponse
%TranslateRecognitionId gives an empty data for household_objects_database_msgs/TranslateRecognitionIdResponse

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'household_objects_database_msgs/TranslateRecognitionIdResponse';
[data.HouseholdObjectsId, info.HouseholdObjectsId] = ros.internal.ros.messages.ros.default_type('int32',1);
[data.SUCCESS, info.SUCCESS] = ros.internal.ros.messages.ros.default_type('int32',1, 0);
[data.IDNOTFOUND, info.IDNOTFOUND] = ros.internal.ros.messages.ros.default_type('int32',1);
[data.DATABASEERROR, info.DATABASEERROR] = ros.internal.ros.messages.ros.default_type('int32',1);
[data.OTHERERROR, info.OTHERERROR] = ros.internal.ros.messages.ros.default_type('int32',1);
[data.Result, info.Result] = ros.internal.ros.messages.ros.default_type('int32',1);
info.MessageType = 'household_objects_database_msgs/TranslateRecognitionIdResponse';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,6);
info.MatPath{1} = 'household_objects_id';
info.MatPath{2} = 'SUCCESS';
info.MatPath{3} = 'ID_NOT_FOUND';
info.MatPath{4} = 'DATABASE_ERROR';
info.MatPath{5} = 'OTHER_ERROR';
info.MatPath{6} = 'result';
