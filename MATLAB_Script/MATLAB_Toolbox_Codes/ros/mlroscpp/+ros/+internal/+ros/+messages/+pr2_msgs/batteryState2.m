function [data, info] = batteryState2
%BatteryState2 gives an empty data for pr2_msgs/BatteryState2

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'pr2_msgs/BatteryState2';
[data.Present, info.Present] = ros.internal.ros.messages.ros.default_type('logical',1);
[data.Charging, info.Charging] = ros.internal.ros.messages.ros.default_type('logical',1);
[data.Discharging, info.Discharging] = ros.internal.ros.messages.ros.default_type('logical',1);
[data.PowerPresent, info.PowerPresent] = ros.internal.ros.messages.ros.default_type('logical',1);
[data.PowerNoGood, info.PowerNoGood] = ros.internal.ros.messages.ros.default_type('logical',1);
[data.Inhibited, info.Inhibited] = ros.internal.ros.messages.ros.default_type('logical',1);
[data.LastBatteryUpdate, info.LastBatteryUpdate] = ros.internal.ros.messages.ros.time;
info.LastBatteryUpdate.MLdataType = 'struct';
[data.BatteryRegister, info.BatteryRegister] = ros.internal.ros.messages.ros.default_type('int16',48);
[data.BatteryUpdateFlag, info.BatteryUpdateFlag] = ros.internal.ros.messages.ros.default_type('logical',48);
[data.BatteryRegisterUpdate, info.BatteryRegisterUpdate] = ros.internal.ros.messages.ros.time;
info.BatteryRegisterUpdate.MLdataType = 'struct';
info.BatteryRegisterUpdate.MaxLen = 48;
info.BatteryRegisterUpdate.MinLen = 48;
val = [];
for i = 1:48
    val = vertcat(data.BatteryRegisterUpdate, val); %#ok<AGROW>
end
data.BatteryRegisterUpdate = val;
info.MessageType = 'pr2_msgs/BatteryState2';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,14);
info.MatPath{1} = 'present';
info.MatPath{2} = 'charging';
info.MatPath{3} = 'discharging';
info.MatPath{4} = 'power_present';
info.MatPath{5} = 'power_no_good';
info.MatPath{6} = 'inhibited';
info.MatPath{7} = 'last_battery_update';
info.MatPath{8} = 'last_battery_update.sec';
info.MatPath{9} = 'last_battery_update.nsec';
info.MatPath{10} = 'battery_register';
info.MatPath{11} = 'battery_update_flag';
info.MatPath{12} = 'battery_register_update';
info.MatPath{13} = 'battery_register_update.sec';
info.MatPath{14} = 'battery_register_update.nsec';
