function [data, info] = alarmSensor
%AlarmSensor gives an empty data for robotnik_msgs/AlarmSensor

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'robotnik_msgs/AlarmSensor';
[data.Id, info.Id] = ros.internal.ros.messages.ros.default_type('int8',1);
[data.Description, info.Description] = ros.internal.ros.messages.ros.char('string',0);
[data.Message, info.Message] = ros.internal.ros.messages.ros.char('string',0);
info.MessageType = 'robotnik_msgs/AlarmSensor';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,3);
info.MatPath{1} = 'id';
info.MatPath{2} = 'description';
info.MatPath{3} = 'message';
