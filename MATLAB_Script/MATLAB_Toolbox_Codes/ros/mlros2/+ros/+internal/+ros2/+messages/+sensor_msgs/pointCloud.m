function [data, info] = pointCloud
%PointCloud gives an empty data for sensor_msgs/PointCloud

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'sensor_msgs/PointCloud';
[data.header, info.header] = ros.internal.ros2.messages.std_msgs.header;
info.header.MLdataType = 'struct';
[data.points, info.points] = ros.internal.ros2.messages.geometry_msgs.point32;
info.points.MLdataType = 'struct';
info.points.MaxLen = NaN;
info.points.MinLen = 0;
[data.channels, info.channels] = ros.internal.ros2.messages.sensor_msgs.channelFloat32;
info.channels.MLdataType = 'struct';
info.channels.MaxLen = NaN;
info.channels.MinLen = 0;
info.MessageType = 'sensor_msgs/PointCloud';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,12);
info.MatPath{1} = 'header';
info.MatPath{2} = 'header.stamp';
info.MatPath{3} = 'header.stamp.sec';
info.MatPath{4} = 'header.stamp.nanosec';
info.MatPath{5} = 'header.frame_id';
info.MatPath{6} = 'points';
info.MatPath{7} = 'points.x';
info.MatPath{8} = 'points.y';
info.MatPath{9} = 'points.z';
info.MatPath{10} = 'channels';
info.MatPath{11} = 'channels.name';
info.MatPath{12} = 'channels.values';
