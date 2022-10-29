function [data, info] = occupancyGridUpdate
%OccupancyGridUpdate gives an empty data for map_msgs/OccupancyGridUpdate

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'map_msgs/OccupancyGridUpdate';
[data.header, info.header] = ros.internal.ros2.messages.std_msgs.header;
info.header.MLdataType = 'struct';
[data.x, info.x] = ros.internal.ros2.messages.ros2.default_type('int32',1,0);
[data.y, info.y] = ros.internal.ros2.messages.ros2.default_type('int32',1,0);
[data.width, info.width] = ros.internal.ros2.messages.ros2.default_type('uint32',1,0);
[data.height, info.height] = ros.internal.ros2.messages.ros2.default_type('uint32',1,0);
[data.data, info.data] = ros.internal.ros2.messages.ros2.default_type('int8',NaN,0);
info.MessageType = 'map_msgs/OccupancyGridUpdate';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,10);
info.MatPath{1} = 'header';
info.MatPath{2} = 'header.stamp';
info.MatPath{3} = 'header.stamp.sec';
info.MatPath{4} = 'header.stamp.nanosec';
info.MatPath{5} = 'header.frame_id';
info.MatPath{6} = 'x';
info.MatPath{7} = 'y';
info.MatPath{8} = 'width';
info.MatPath{9} = 'height';
info.MatPath{10} = 'data';
