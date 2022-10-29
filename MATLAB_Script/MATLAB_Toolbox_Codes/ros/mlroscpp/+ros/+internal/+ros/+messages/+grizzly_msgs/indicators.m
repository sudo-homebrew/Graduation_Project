function [data, info] = indicators
%Indicators gives an empty data for grizzly_msgs/Indicators

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'grizzly_msgs/Indicators';
[data.INDICATORON, info.INDICATORON] = ros.internal.ros.messages.ros.default_type('uint8',1, 255);
[data.INDICATORFLASH, info.INDICATORFLASH] = ros.internal.ros.messages.ros.default_type('uint8',1, 15);
[data.INDICATOROFF, info.INDICATOROFF] = ros.internal.ros.messages.ros.default_type('uint8',1, 0);
[data.PositionLight, info.PositionLight] = ros.internal.ros.messages.ros.default_type('uint8',1);
[data.AutopilotLight, info.AutopilotLight] = ros.internal.ros.messages.ros.default_type('uint8',1);
[data.BatteryLight, info.BatteryLight] = ros.internal.ros.messages.ros.default_type('uint8',1);
info.MessageType = 'grizzly_msgs/Indicators';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,6);
info.MatPath{1} = 'INDICATOR_ON';
info.MatPath{2} = 'INDICATOR_FLASH';
info.MatPath{3} = 'INDICATOR_OFF';
info.MatPath{4} = 'position_light';
info.MatPath{5} = 'autopilot_light';
info.MatPath{6} = 'battery_light';
