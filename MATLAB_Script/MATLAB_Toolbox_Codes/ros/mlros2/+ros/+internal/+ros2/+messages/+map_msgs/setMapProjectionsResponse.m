function [data, info] = setMapProjectionsResponse
%SetMapProjections gives an empty data for map_msgs/SetMapProjectionsResponse

% Copyright 2019-2021 The MathWorks, Inc.
data = struct();
data.MessageType = 'map_msgs/SetMapProjectionsResponse';
[data.projected_maps_info, info.projected_maps_info] = ros.internal.ros2.messages.map_msgs.projectedMapInfo;
info.projected_maps_info.MLdataType = 'struct';
info.projected_maps_info.MaxLen = NaN;
info.projected_maps_info.MinLen = 0;
info.MessageType = 'map_msgs/SetMapProjectionsResponse';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,8);
info.MatPath{1} = 'projected_maps_info';
info.MatPath{2} = 'projected_maps_info.frame_id';
info.MatPath{3} = 'projected_maps_info.x';
info.MatPath{4} = 'projected_maps_info.y';
info.MatPath{5} = 'projected_maps_info.width';
info.MatPath{6} = 'projected_maps_info.height';
info.MatPath{7} = 'projected_maps_info.min_z';
info.MatPath{8} = 'projected_maps_info.max_z';
