function [data, info] = testValue
%TestValue gives an empty data for pr2_self_test_msgs/TestValue

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'pr2_self_test_msgs/TestValue';
[data.Key, info.Key] = ros.internal.ros.messages.ros.char('string',0);
[data.Value, info.Value] = ros.internal.ros.messages.ros.char('string',0);
[data.Min, info.Min] = ros.internal.ros.messages.ros.char('string',0);
[data.Max, info.Max] = ros.internal.ros.messages.ros.char('string',0);
info.MessageType = 'pr2_self_test_msgs/TestValue';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,4);
info.MatPath{1} = 'key';
info.MatPath{2} = 'value';
info.MatPath{3} = 'min';
info.MatPath{4} = 'max';
