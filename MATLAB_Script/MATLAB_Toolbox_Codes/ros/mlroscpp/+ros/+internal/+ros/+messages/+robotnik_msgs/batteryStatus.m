function [data, info] = batteryStatus
%BatteryStatus gives an empty data for robotnik_msgs/BatteryStatus

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'robotnik_msgs/BatteryStatus';
[data.Voltage, info.Voltage] = ros.internal.ros.messages.ros.default_type('single',1);
[data.Current, info.Current] = ros.internal.ros.messages.ros.default_type('single',1);
[data.Level, info.Level] = ros.internal.ros.messages.ros.default_type('single',1);
[data.TimeRemaining, info.TimeRemaining] = ros.internal.ros.messages.ros.default_type('uint32',1);
[data.TimeCharging, info.TimeCharging] = ros.internal.ros.messages.ros.default_type('uint32',1);
[data.IsCharging, info.IsCharging] = ros.internal.ros.messages.ros.default_type('logical',1);
info.MessageType = 'robotnik_msgs/BatteryStatus';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,6);
info.MatPath{1} = 'voltage';
info.MatPath{2} = 'current';
info.MatPath{3} = 'level';
info.MatPath{4} = 'time_remaining';
info.MatPath{5} = 'time_charging';
info.MatPath{6} = 'is_charging';
