function [data, info] = get_alarmsResponse
%get_alarms gives an empty data for robotnik_msgs/get_alarmsResponse

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'robotnik_msgs/get_alarmsResponse';
[data.Alarms, info.Alarms] = ros.internal.ros.messages.robotnik_msgs.alarmsmonitor;
info.Alarms.MLdataType = 'struct';
info.MessageType = 'robotnik_msgs/get_alarmsResponse';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,9);
info.MatPath{1} = 'alarms';
info.MatPath{2} = 'alarms.alarms';
info.MatPath{3} = 'alarms.alarms.type';
info.MatPath{4} = 'alarms.alarms.display';
info.MatPath{5} = 'alarms.alarms.component';
info.MatPath{6} = 'alarms.alarms.hmi';
info.MatPath{7} = 'alarms.alarms.group';
info.MatPath{8} = 'alarms.alarms.text';
info.MatPath{9} = 'alarms.alarms.seconds_active';
