function [data, info] = getModelMeshRequest
%GetModelMesh gives an empty data for household_objects_database_msgs/GetModelMeshRequest

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'household_objects_database_msgs/GetModelMeshRequest';
[data.ModelId, info.ModelId] = ros.internal.ros.messages.ros.default_type('int32',1);
info.MessageType = 'household_objects_database_msgs/GetModelMeshRequest';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,1);
info.MatPath{1} = 'model_id';
