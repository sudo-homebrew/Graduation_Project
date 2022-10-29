function [data, info] = getModelListRequest
%GetModelList gives an empty data for household_objects_database_msgs/GetModelListRequest

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'household_objects_database_msgs/GetModelListRequest';
[data.ModelSet, info.ModelSet] = ros.internal.ros.messages.ros.char('string',0);
info.MessageType = 'household_objects_database_msgs/GetModelListRequest';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,1);
info.MatPath{1} = 'model_set';
