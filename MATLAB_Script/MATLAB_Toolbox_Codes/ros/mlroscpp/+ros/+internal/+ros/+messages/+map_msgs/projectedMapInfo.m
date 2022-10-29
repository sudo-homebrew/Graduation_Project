function [data, info] = projectedMapInfo
%ProjectedMapInfo gives an empty data for map_msgs/ProjectedMapInfo

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'map_msgs/ProjectedMapInfo';
[data.FrameId, info.FrameId] = ros.internal.ros.messages.ros.char('string',0);
[data.X, info.X] = ros.internal.ros.messages.ros.default_type('double',1);
[data.Y, info.Y] = ros.internal.ros.messages.ros.default_type('double',1);
[data.Width, info.Width] = ros.internal.ros.messages.ros.default_type('double',1);
[data.Height, info.Height] = ros.internal.ros.messages.ros.default_type('double',1);
[data.MinZ, info.MinZ] = ros.internal.ros.messages.ros.default_type('double',1);
[data.MaxZ, info.MaxZ] = ros.internal.ros.messages.ros.default_type('double',1);
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
