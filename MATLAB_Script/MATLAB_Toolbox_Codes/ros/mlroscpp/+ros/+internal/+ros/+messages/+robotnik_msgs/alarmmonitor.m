function [data, info] = alarmmonitor
%alarmmonitor gives an empty data for robotnik_msgs/alarmmonitor

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'robotnik_msgs/alarmmonitor';
[data.Type, info.Type] = ros.internal.ros.messages.ros.char('string',0);
[data.Display, info.Display] = ros.internal.ros.messages.ros.default_type('int32',1);
[data.Component, info.Component] = ros.internal.ros.messages.ros.char('string',0);
[data.Hmi, info.Hmi] = ros.internal.ros.messages.ros.default_type('logical',1);
[data.Group, info.Group] = ros.internal.ros.messages.ros.char('string',0);
[data.Text, info.Text] = ros.internal.ros.messages.ros.char('string',0);
[data.SecondsActive, info.SecondsActive] = ros.internal.ros.messages.ros.default_type('uint64',1);
info.MessageType = 'robotnik_msgs/alarmmonitor';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,7);
info.MatPath{1} = 'type';
info.MatPath{2} = 'display';
info.MatPath{3} = 'component';
info.MatPath{4} = 'hmi';
info.MatPath{5} = 'group';
info.MatPath{6} = 'text';
info.MatPath{7} = 'seconds_active';
