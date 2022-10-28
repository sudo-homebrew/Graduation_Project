function [data, info] = pointCloud
%PointCloud gives an empty data for sensor_msgs/PointCloud

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'sensor_msgs/PointCloud';
[data.Header, info.Header] = ros.internal.ros.messages.std_msgs.header;
info.Header.MLdataType = 'struct';
[data.Points, info.Points] = ros.internal.ros.messages.geometry_msgs.point32;
info.Points.MLdataType = 'struct';
info.Points.MaxLen = NaN;
info.Points.MinLen = 0;
data.Points = data.Points([],1);
[data.Channels, info.Channels] = ros.internal.ros.messages.sensor_msgs.channelFloat32;
info.Channels.MLdataType = 'struct';
info.Channels.MaxLen = NaN;
info.Channels.MinLen = 0;
data.Channels = data.Channels([],1);
info.MessageType = 'sensor_msgs/PointCloud';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,13);
info.MatPath{1} = 'header';
info.MatPath{2} = 'header.seq';
info.MatPath{3} = 'header.stamp';
info.MatPath{4} = 'header.stamp.sec';
info.MatPath{5} = 'header.stamp.nsec';
info.MatPath{6} = 'header.frame_id';
info.MatPath{7} = 'points';
info.MatPath{8} = 'points.x';
info.MatPath{9} = 'points.y';
info.MatPath{10} = 'points.z';
info.MatPath{11} = 'channels';
info.MatPath{12} = 'channels.name';
info.MatPath{13} = 'channels.values';
