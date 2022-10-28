function [data, info] = channelFloat32
%ChannelFloat32 gives an empty data for sensor_msgs/ChannelFloat32

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'sensor_msgs/ChannelFloat32';
[data.Name, info.Name] = ros.internal.ros.messages.ros.char('string',0);
[data.Values, info.Values] = ros.internal.ros.messages.ros.default_type('single',NaN);
info.MessageType = 'sensor_msgs/ChannelFloat32';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,2);
info.MatPath{1} = 'name';
info.MatPath{2} = 'values';
