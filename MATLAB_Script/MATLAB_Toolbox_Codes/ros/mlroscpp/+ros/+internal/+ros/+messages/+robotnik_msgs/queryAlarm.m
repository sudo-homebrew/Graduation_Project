function [data, info] = queryAlarm
%QueryAlarm gives an empty data for robotnik_msgs/QueryAlarm

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'robotnik_msgs/QueryAlarm';
[data.AlmId, info.AlmId] = ros.internal.ros.messages.ros.default_type('int32',1);
[data.AlmComponent, info.AlmComponent] = ros.internal.ros.messages.ros.char('string',0);
[data.AlmType, info.AlmType] = ros.internal.ros.messages.ros.char('string',0);
[data.AlmGroup, info.AlmGroup] = ros.internal.ros.messages.ros.char('string',0);
[data.AlmDescription, info.AlmDescription] = ros.internal.ros.messages.ros.char('string',0);
[data.AlmStartTime, info.AlmStartTime] = ros.internal.ros.messages.ros.char('string',0);
[data.AlmEndTime, info.AlmEndTime] = ros.internal.ros.messages.ros.char('string',0);
info.MessageType = 'robotnik_msgs/QueryAlarm';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,7);
info.MatPath{1} = 'alm_id';
info.MatPath{2} = 'alm_component';
info.MatPath{3} = 'alm_type';
info.MatPath{4} = 'alm_group';
info.MatPath{5} = 'alm_description';
info.MatPath{6} = 'alm_start_time';
info.MatPath{7} = 'alm_end_time';
