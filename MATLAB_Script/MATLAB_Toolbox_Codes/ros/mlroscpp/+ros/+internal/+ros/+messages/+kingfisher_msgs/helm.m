function [data, info] = helm
%Helm gives an empty data for kingfisher_msgs/Helm

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'kingfisher_msgs/Helm';
[data.Thrust, info.Thrust] = ros.internal.ros.messages.ros.default_type('single',1);
[data.YawRate, info.YawRate] = ros.internal.ros.messages.ros.default_type('single',1);
info.MessageType = 'kingfisher_msgs/Helm';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,2);
info.MatPath{1} = 'thrust';
info.MatPath{2} = 'yaw_rate';
