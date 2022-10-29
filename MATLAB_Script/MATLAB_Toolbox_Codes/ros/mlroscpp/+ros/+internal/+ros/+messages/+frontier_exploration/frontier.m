function [data, info] = frontier
%Frontier gives an empty data for frontier_exploration/Frontier

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'frontier_exploration/Frontier';
[data.Size, info.Size] = ros.internal.ros.messages.ros.default_type('uint32',1);
[data.MinDistance, info.MinDistance] = ros.internal.ros.messages.ros.default_type('double',1);
[data.Initial, info.Initial] = ros.internal.ros.messages.geometry_msgs.point;
info.Initial.MLdataType = 'struct';
[data.Centroid, info.Centroid] = ros.internal.ros.messages.geometry_msgs.point;
info.Centroid.MLdataType = 'struct';
[data.Middle, info.Middle] = ros.internal.ros.messages.geometry_msgs.point;
info.Middle.MLdataType = 'struct';
info.MessageType = 'frontier_exploration/Frontier';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,14);
info.MatPath{1} = 'size';
info.MatPath{2} = 'min_distance';
info.MatPath{3} = 'initial';
info.MatPath{4} = 'initial.x';
info.MatPath{5} = 'initial.y';
info.MatPath{6} = 'initial.z';
info.MatPath{7} = 'centroid';
info.MatPath{8} = 'centroid.x';
info.MatPath{9} = 'centroid.y';
info.MatPath{10} = 'centroid.z';
info.MatPath{11} = 'middle';
info.MatPath{12} = 'middle.x';
info.MatPath{13} = 'middle.y';
info.MatPath{14} = 'middle.z';
