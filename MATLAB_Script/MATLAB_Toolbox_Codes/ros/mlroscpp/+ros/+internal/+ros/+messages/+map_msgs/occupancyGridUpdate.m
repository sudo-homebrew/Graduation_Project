function [data, info] = occupancyGridUpdate
%OccupancyGridUpdate gives an empty data for map_msgs/OccupancyGridUpdate

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'map_msgs/OccupancyGridUpdate';
[data.Header, info.Header] = ros.internal.ros.messages.std_msgs.header;
info.Header.MLdataType = 'struct';
[data.X, info.X] = ros.internal.ros.messages.ros.default_type('int32',1);
[data.Y, info.Y] = ros.internal.ros.messages.ros.default_type('int32',1);
[data.Width, info.Width] = ros.internal.ros.messages.ros.default_type('uint32',1);
[data.Height, info.Height] = ros.internal.ros.messages.ros.default_type('uint32',1);
[data.Data, info.Data] = ros.internal.ros.messages.ros.default_type('int8',NaN);
info.MessageType = 'map_msgs/OccupancyGridUpdate';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,11);
info.MatPath{1} = 'header';
info.MatPath{2} = 'header.seq';
info.MatPath{3} = 'header.stamp';
info.MatPath{4} = 'header.stamp.sec';
info.MatPath{5} = 'header.stamp.nsec';
info.MatPath{6} = 'header.frame_id';
info.MatPath{7} = 'x';
info.MatPath{8} = 'y';
info.MatPath{9} = 'width';
info.MatPath{10} = 'height';
info.MatPath{11} = 'data';
