function [data, info] = getModelMeshResponse
%GetModelMesh gives an empty data for household_objects_database_msgs/GetModelMeshResponse

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'household_objects_database_msgs/GetModelMeshResponse';
[data.ReturnCode, info.ReturnCode] = ros.internal.ros.messages.household_objects_database_msgs.databaseReturnCode;
info.ReturnCode.MLdataType = 'struct';
[data.Mesh, info.Mesh] = ros.internal.ros.messages.shape_msgs.mesh;
info.Mesh.MLdataType = 'struct';
info.MessageType = 'household_objects_database_msgs/GetModelMeshResponse';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,13);
info.MatPath{1} = 'return_code';
info.MatPath{2} = 'return_code.UNKNOWN_ERROR';
info.MatPath{3} = 'return_code.DATABASE_NOT_CONNECTED';
info.MatPath{4} = 'return_code.DATABASE_QUERY_ERROR';
info.MatPath{5} = 'return_code.SUCCESS';
info.MatPath{6} = 'return_code.code';
info.MatPath{7} = 'mesh';
info.MatPath{8} = 'mesh.triangles';
info.MatPath{9} = 'mesh.triangles.vertex_indices';
info.MatPath{10} = 'mesh.vertices';
info.MatPath{11} = 'mesh.vertices.x';
info.MatPath{12} = 'mesh.vertices.y';
info.MatPath{13} = 'mesh.vertices.z';
