function [data, info] = mesh
%Mesh gives an empty data for shape_msgs/Mesh

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'shape_msgs/Mesh';
[data.Triangles, info.Triangles] = ros.internal.ros.messages.shape_msgs.meshTriangle;
info.Triangles.MLdataType = 'struct';
info.Triangles.MaxLen = NaN;
info.Triangles.MinLen = 0;
data.Triangles = data.Triangles([],1);
[data.Vertices, info.Vertices] = ros.internal.ros.messages.geometry_msgs.point;
info.Vertices.MLdataType = 'struct';
info.Vertices.MaxLen = NaN;
info.Vertices.MinLen = 0;
data.Vertices = data.Vertices([],1);
info.MessageType = 'shape_msgs/Mesh';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,6);
info.MatPath{1} = 'triangles';
info.MatPath{2} = 'triangles.vertex_indices';
info.MatPath{3} = 'vertices';
info.MatPath{4} = 'vertices.x';
info.MatPath{5} = 'vertices.y';
info.MatPath{6} = 'vertices.z';
