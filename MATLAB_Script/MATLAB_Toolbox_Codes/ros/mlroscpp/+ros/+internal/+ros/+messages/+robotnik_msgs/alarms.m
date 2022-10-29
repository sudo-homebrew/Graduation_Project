function [data, info] = alarms
%Alarms gives an empty data for robotnik_msgs/Alarms

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'robotnik_msgs/Alarms';
[data.Alarms_, info.Alarms_] = ros.internal.ros.messages.robotnik_msgs.alarmSensor;
info.Alarms_.MLdataType = 'struct';
info.Alarms_.MaxLen = NaN;
info.Alarms_.MinLen = 0;
data.Alarms_ = data.Alarms_([],1);
info.MessageType = 'robotnik_msgs/Alarms';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,4);
info.MatPath{1} = 'alarms';
info.MatPath{2} = 'alarms.id';
info.MatPath{3} = 'alarms.description';
info.MatPath{4} = 'alarms.message';
