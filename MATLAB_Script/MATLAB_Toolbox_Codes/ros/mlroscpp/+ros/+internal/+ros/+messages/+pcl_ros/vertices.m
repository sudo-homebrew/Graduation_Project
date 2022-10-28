function [data, info] = vertices
%Vertices gives an empty data for pcl_ros/Vertices

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'pcl_ros/Vertices';
[data.Vertices_, info.Vertices_] = ros.internal.ros.messages.ros.default_type('uint32',NaN);
info.MessageType = 'pcl_ros/Vertices';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,1);
info.MatPath{1} = 'vertices';
