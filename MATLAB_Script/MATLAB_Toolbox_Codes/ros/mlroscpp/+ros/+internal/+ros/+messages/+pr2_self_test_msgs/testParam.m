function [data, info] = testParam
%TestParam gives an empty data for pr2_self_test_msgs/TestParam

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'pr2_self_test_msgs/TestParam';
[data.Key, info.Key] = ros.internal.ros.messages.ros.char('string',0);
[data.Value, info.Value] = ros.internal.ros.messages.ros.char('string',0);
info.MessageType = 'pr2_self_test_msgs/TestParam';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,2);
info.MatPath{1} = 'key';
info.MatPath{2} = 'value';
