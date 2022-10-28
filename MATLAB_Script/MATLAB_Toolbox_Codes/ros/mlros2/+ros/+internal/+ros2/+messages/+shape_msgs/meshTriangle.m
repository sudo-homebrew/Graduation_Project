function [data, info] = meshTriangle
%MeshTriangle gives an empty data for shape_msgs/MeshTriangle

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'shape_msgs/MeshTriangle';
[data.vertex_indices, info.vertex_indices] = ros.internal.ros2.messages.ros2.default_type('uint32',3,0);
info.MessageType = 'shape_msgs/MeshTriangle';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,1);
info.MatPath{1} = 'vertex_indices';
