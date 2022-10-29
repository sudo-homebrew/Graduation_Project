function [data, info] = projectedMapInfo
%ProjectedMapInfo gives an empty data for map_msgs/ProjectedMapInfo

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'map_msgs/ProjectedMapInfo';
[data.frame_id, info.frame_id] = ros.internal.ros2.messages.ros2.char('string',1,NaN,0);
[data.x, info.x] = ros.internal.ros2.messages.ros2.default_type('double',1,0);
[data.y, info.y] = ros.internal.ros2.messages.ros2.default_type('double',1,0);
[data.width, info.width] = ros.internal.ros2.messages.ros2.default_type('double',1,0);
[data.height, info.height] = ros.internal.ros2.messages.ros2.default_type('double',1,0);
[data.min_z, info.min_z] = ros.internal.ros2.messages.ros2.default_type('double',1,0);
[data.max_z, info.max_z] = ros.internal.ros2.messages.ros2.default_type('double',1,0);
info.MessageType = 'map_msgs/ProjectedMapInfo';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,7);
info.MatPath{1} = 'frame_id';
info.MatPath{2} = 'x';
info.MatPath{3} = 'y';
info.MatPath{4} = 'width';
info.MatPath{5} = 'height';
info.MatPath{6} = 'min_z';
info.MatPath{7} = 'max_z';
