function [data, info] = laserEcho
%LaserEcho gives an empty data for sensor_msgs/LaserEcho

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'sensor_msgs/LaserEcho';
[data.echoes, info.echoes] = ros.internal.ros2.messages.ros2.default_type('single',NaN,0);
info.MessageType = 'sensor_msgs/LaserEcho';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,1);
info.MatPath{1} = 'echoes';
