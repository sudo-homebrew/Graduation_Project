function [data, info] = batteryState
%BatteryState gives an empty data for pr2_msgs/BatteryState

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'pr2_msgs/BatteryState';
[data.LastTimeBattery, info.LastTimeBattery] = ros.internal.ros.messages.ros.default_type('int32',1);
[data.BatReg, info.BatReg] = ros.internal.ros.messages.ros.default_type('uint16',48);
[data.BatRegFlag, info.BatRegFlag] = ros.internal.ros.messages.ros.default_type('uint16',48);
[data.BatRegTime, info.BatRegTime] = ros.internal.ros.messages.ros.default_type('int32',48);
info.MessageType = 'pr2_msgs/BatteryState';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,4);
info.MatPath{1} = 'lastTimeBattery';
info.MatPath{2} = 'batReg';
info.MatPath{3} = 'batRegFlag';
info.MatPath{4} = 'batRegTime';
