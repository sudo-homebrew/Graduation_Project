function [data, info] = powerSystemEvent
%PowerSystemEvent gives an empty data for kobuki_msgs/PowerSystemEvent

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'kobuki_msgs/PowerSystemEvent';
[data.UNPLUGGED, info.UNPLUGGED] = ros.internal.ros.messages.ros.default_type('uint8',1, 0);
[data.PLUGGEDTOADAPTER, info.PLUGGEDTOADAPTER] = ros.internal.ros.messages.ros.default_type('uint8',1, 1);
[data.PLUGGEDTODOCKBASE, info.PLUGGEDTODOCKBASE] = ros.internal.ros.messages.ros.default_type('uint8',1, 2);
[data.CHARGECOMPLETED, info.CHARGECOMPLETED] = ros.internal.ros.messages.ros.default_type('uint8',1, 3);
[data.BATTERYLOW, info.BATTERYLOW] = ros.internal.ros.messages.ros.default_type('uint8',1, 4);
[data.BATTERYCRITICAL, info.BATTERYCRITICAL] = ros.internal.ros.messages.ros.default_type('uint8',1, 5);
[data.Event, info.Event] = ros.internal.ros.messages.ros.default_type('uint8',1);
info.MessageType = 'kobuki_msgs/PowerSystemEvent';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,7);
info.MatPath{1} = 'UNPLUGGED';
info.MatPath{2} = 'PLUGGED_TO_ADAPTER';
info.MatPath{3} = 'PLUGGED_TO_DOCKBASE';
info.MatPath{4} = 'CHARGE_COMPLETED';
info.MatPath{5} = 'BATTERY_LOW';
info.MatPath{6} = 'BATTERY_CRITICAL';
info.MatPath{7} = 'event';
