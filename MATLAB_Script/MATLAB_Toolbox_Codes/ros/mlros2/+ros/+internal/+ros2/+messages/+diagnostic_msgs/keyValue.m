function [data, info] = keyValue
%KeyValue gives an empty data for diagnostic_msgs/KeyValue

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'diagnostic_msgs/KeyValue';
[data.key, info.key] = ros.internal.ros2.messages.ros2.char('string',1,NaN,0);
[data.value, info.value] = ros.internal.ros2.messages.ros2.char('string',1,NaN,0);
info.MessageType = 'diagnostic_msgs/KeyValue';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,2);
info.MatPath{1} = 'key';
info.MatPath{2} = 'value';
