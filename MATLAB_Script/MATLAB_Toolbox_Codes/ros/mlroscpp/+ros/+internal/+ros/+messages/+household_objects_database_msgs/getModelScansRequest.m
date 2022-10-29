function [data, info] = getModelScansRequest
%GetModelScans gives an empty data for household_objects_database_msgs/GetModelScansRequest

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'household_objects_database_msgs/GetModelScansRequest';
[data.ModelId, info.ModelId] = ros.internal.ros.messages.ros.default_type('int32',1);
[data.ScanSource, info.ScanSource] = ros.internal.ros.messages.ros.char('string',0);
info.MessageType = 'household_objects_database_msgs/GetModelScansRequest';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,2);
info.MatPath{1} = 'model_id';
info.MatPath{2} = 'scan_source';
