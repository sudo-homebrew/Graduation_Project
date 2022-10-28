function [data, info] = costSource
%CostSource gives an empty data for moveit_msgs/CostSource

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'moveit_msgs/CostSource';
[data.CostDensity, info.CostDensity] = ros.internal.ros.messages.ros.default_type('double',1);
[data.AabbMin, info.AabbMin] = ros.internal.ros.messages.geometry_msgs.vector3;
info.AabbMin.MLdataType = 'struct';
[data.AabbMax, info.AabbMax] = ros.internal.ros.messages.geometry_msgs.vector3;
info.AabbMax.MLdataType = 'struct';
info.MessageType = 'moveit_msgs/CostSource';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,9);
info.MatPath{1} = 'cost_density';
info.MatPath{2} = 'aabb_min';
info.MatPath{3} = 'aabb_min.x';
info.MatPath{4} = 'aabb_min.y';
info.MatPath{5} = 'aabb_min.z';
info.MatPath{6} = 'aabb_max';
info.MatPath{7} = 'aabb_max.x';
info.MatPath{8} = 'aabb_max.y';
info.MatPath{9} = 'aabb_max.z';
