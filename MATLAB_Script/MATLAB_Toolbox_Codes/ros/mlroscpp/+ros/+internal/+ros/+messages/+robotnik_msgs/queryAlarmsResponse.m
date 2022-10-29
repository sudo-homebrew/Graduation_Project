function [data, info] = queryAlarmsResponse
%QueryAlarms gives an empty data for robotnik_msgs/QueryAlarmsResponse

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'robotnik_msgs/QueryAlarmsResponse';
[data.Alarms, info.Alarms] = ros.internal.ros.messages.robotnik_msgs.queryAlarm;
info.Alarms.MLdataType = 'struct';
info.Alarms.MaxLen = NaN;
info.Alarms.MinLen = 0;
data.Alarms = data.Alarms([],1);
[data.Success, info.Success] = ros.internal.ros.messages.ros.default_type('logical',1);
[data.Msg, info.Msg] = ros.internal.ros.messages.ros.char('string',0);
info.MessageType = 'robotnik_msgs/QueryAlarmsResponse';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,10);
info.MatPath{1} = 'alarms';
info.MatPath{2} = 'alarms.alm_id';
info.MatPath{3} = 'alarms.alm_component';
info.MatPath{4} = 'alarms.alm_type';
info.MatPath{5} = 'alarms.alm_group';
info.MatPath{6} = 'alarms.alm_description';
info.MatPath{7} = 'alarms.alm_start_time';
info.MatPath{8} = 'alarms.alm_end_time';
info.MatPath{9} = 'success';
info.MatPath{10} = 'msg';
