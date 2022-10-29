function [data, info] = mesh
%Mesh gives an empty data for shape_msgs/Mesh

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'shape_msgs/Mesh';
[data.triangles, info.triangles] = ros.internal.ros2.messages.shape_msgs.meshTriangle;
info.triangles.MLdataType = 'struct';
info.triangles.MaxLen = NaN;
info.triangles.MinLen = 0;
[data.vertices, info.vertices] = ros.internal.ros2.messages.geometry_msgs.point;
info.vertices.MLdataType = 'struct';
info.vertices.MaxLen = NaN;
info.vertices.MinLen = 0;
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
