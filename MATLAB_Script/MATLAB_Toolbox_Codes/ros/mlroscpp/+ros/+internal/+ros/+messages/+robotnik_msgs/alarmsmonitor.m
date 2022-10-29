function [data, info] = alarmsmonitor
%alarmsmonitor gives an empty data for robotnik_msgs/alarmsmonitor

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'robotnik_msgs/alarmsmonitor';
[data.Alarms, info.Alarms] = ros.internal.ros.messages.robotnik_msgs.alarmmonitor;
info.Alarms.MLdataType = 'struct';
info.Alarms.MaxLen = NaN;
info.Alarms.MinLen = 0;
data.Alarms = data.Alarms([],1);
info.MessageType = 'robotnik_msgs/alarmsmonitor';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,8);
info.MatPath{1} = 'alarms';
info.MatPath{2} = 'alarms.type';
info.MatPath{3} = 'alarms.display';
info.MatPath{4} = 'alarms.component';
info.MatPath{5} = 'alarms.hmi';
info.MatPath{6} = 'alarms.group';
info.MatPath{7} = 'alarms.text';
info.MatPath{8} = 'alarms.seconds_active';
