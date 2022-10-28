function [data, info] = batteryDockingStatus
%BatteryDockingStatus gives an empty data for robotnik_msgs/BatteryDockingStatus

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'robotnik_msgs/BatteryDockingStatus';
[data.MODEDISABLED, info.MODEDISABLED] = ros.internal.ros.messages.ros.char('string',0);
[data.MODEDISABLED, info.MODEDISABLED] = ros.internal.ros.messages.ros.char('string',1,'disabled');
[data.MODEAUTOHW, info.MODEAUTOHW] = ros.internal.ros.messages.ros.char('string',0);
[data.MODEAUTOHW, info.MODEAUTOHW] = ros.internal.ros.messages.ros.char('string',1,'automatic_hw');
[data.MODEAUTOSW, info.MODEAUTOSW] = ros.internal.ros.messages.ros.char('string',0);
[data.MODEAUTOSW, info.MODEAUTOSW] = ros.internal.ros.messages.ros.char('string',1,'automatic_sw');
[data.MODEMANUALSW, info.MODEMANUALSW] = ros.internal.ros.messages.ros.char('string',0);
[data.MODEMANUALSW, info.MODEMANUALSW] = ros.internal.ros.messages.ros.char('string',1,'manual_sw');
[data.OperationMode, info.OperationMode] = ros.internal.ros.messages.ros.char('string',0);
[data.ContactRelayStatus, info.ContactRelayStatus] = ros.internal.ros.messages.ros.default_type('logical',1);
[data.ChargerRelayStatus, info.ChargerRelayStatus] = ros.internal.ros.messages.ros.default_type('logical',1);
info.MessageType = 'robotnik_msgs/BatteryDockingStatus';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,7);
info.MatPath{1} = 'MODE_DISABLED';
info.MatPath{2} = 'MODE_AUTO_HW';
info.MatPath{3} = 'MODE_AUTO_SW';
info.MatPath{4} = 'MODE_MANUAL_SW';
info.MatPath{5} = 'operation_mode';
info.MatPath{6} = 'contact_relay_status';
info.MatPath{7} = 'charger_relay_status';
