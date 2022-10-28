function [data, info] = power
%Power gives an empty data for kingfisher_msgs/Power

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'kingfisher_msgs/Power';
[data.Uptime, info.Uptime] = ros.internal.ros.messages.ros.duration;
info.Uptime.MLdataType = 'struct';
[data.UserPower, info.UserPower] = ros.internal.ros.messages.ros.default_type('single',1);
[data.UserPowerTotal, info.UserPowerTotal] = ros.internal.ros.messages.ros.default_type('single',1);
[data.MotorPowerTotal, info.MotorPowerTotal] = ros.internal.ros.messages.ros.default_type('single',1);
info.MessageType = 'kingfisher_msgs/Power';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,6);
info.MatPath{1} = 'uptime';
info.MatPath{2} = 'uptime.sec';
info.MatPath{3} = 'uptime.nsec';
info.MatPath{4} = 'user_power';
info.MatPath{5} = 'user_power_total';
info.MatPath{6} = 'motor_power_total';
