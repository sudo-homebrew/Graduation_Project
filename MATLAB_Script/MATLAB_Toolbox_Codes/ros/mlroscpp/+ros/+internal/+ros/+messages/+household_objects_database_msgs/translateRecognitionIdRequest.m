function [data, info] = translateRecognitionIdRequest
%TranslateRecognitionId gives an empty data for household_objects_database_msgs/TranslateRecognitionIdRequest

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'household_objects_database_msgs/TranslateRecognitionIdRequest';
[data.RecognitionId, info.RecognitionId] = ros.internal.ros.messages.ros.char('string',0);
info.MessageType = 'household_objects_database_msgs/TranslateRecognitionIdRequest';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,1);
info.MatPath{1} = 'recognition_id';
