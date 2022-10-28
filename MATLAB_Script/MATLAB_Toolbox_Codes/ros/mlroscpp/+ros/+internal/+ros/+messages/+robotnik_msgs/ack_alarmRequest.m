function [data, info] = ack_alarmRequest
%ack_alarm gives an empty data for robotnik_msgs/ack_alarmRequest

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'robotnik_msgs/ack_alarmRequest';
[data.Type, info.Type] = ros.internal.ros.messages.ros.char('string',0);
[data.Group, info.Group] = ros.internal.ros.messages.ros.char('string',0);
info.MessageType = 'robotnik_msgs/ack_alarmRequest';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,2);
info.MatPath{1} = 'type';
info.MatPath{2} = 'group';
