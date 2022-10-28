function [data, info] = channelFloat32
%ChannelFloat32 gives an empty data for sensor_msgs/ChannelFloat32

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'sensor_msgs/ChannelFloat32';
[data.name, info.name] = ros.internal.ros2.messages.ros2.char('string',1,NaN,0);
[data.values, info.values] = ros.internal.ros2.messages.ros2.default_type('single',NaN,0);
info.MessageType = 'sensor_msgs/ChannelFloat32';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,2);
info.MatPath{1} = 'name';
info.MatPath{2} = 'values';
