function [data, info] = wStrings
%WStrings gives an empty data for test_msgs/WStrings

% Copyright 2019-2021 The MathWorks, Inc.
data = struct();
data.MessageType = 'test_msgs/WStrings';
[data.wstring_value, info.wstring_value] = ros.internal.ros2.messages.ros2.char('string',1,NaN,0);
[data.wstring_value_default1, info.wstring_value_default1] = ros.internal.ros2.messages.ros2.char('string',1,NaN,0,NaN, 'Hello world!');
[data.wstring_value_default2, info.wstring_value_default2] = ros.internal.ros2.messages.ros2.char('string',1,NaN,0,NaN, 'HellÃ¶ wÃ¶rld!');
[data.wstring_value_default3, info.wstring_value_default3] = ros.internal.ros2.messages.ros2.char('string',1,NaN,0,NaN, 'ãã­ã¼ã¯ã¼ã«ã');
[data.array_of_wstrings, info.array_of_wstrings] = ros.internal.ros2.messages.ros2.char('string',3,NaN,0);
[data.bounded_sequence_of_wstrings, info.bounded_sequence_of_wstrings] = ros.internal.ros2.messages.ros2.char('string',3,NaN,1);
[data.unbounded_sequence_of_wstrings, info.unbounded_sequence_of_wstrings] = ros.internal.ros2.messages.ros2.char('string',NaN,NaN,0);
info.MessageType = 'test_msgs/WStrings';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,7);
info.MatPath{1} = 'wstring_value';
info.MatPath{2} = 'wstring_value_default1';
info.MatPath{3} = 'wstring_value_default2';
info.MatPath{4} = 'wstring_value_default3';
info.MatPath{5} = 'array_of_wstrings';
info.MatPath{6} = 'bounded_sequence_of_wstrings';
info.MatPath{7} = 'unbounded_sequence_of_wstrings';
