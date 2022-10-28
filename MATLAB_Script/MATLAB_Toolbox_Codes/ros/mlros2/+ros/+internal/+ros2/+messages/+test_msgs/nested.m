function [data, info] = nested
%Nested gives an empty data for test_msgs/Nested

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'test_msgs/Nested';
[data.basic_types_value, info.basic_types_value] = ros.internal.ros2.messages.test_msgs.basicTypes;
info.basic_types_value.MLdataType = 'struct';
info.MessageType = 'test_msgs/Nested';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,14);
info.MatPath{1} = 'basic_types_value';
info.MatPath{2} = 'basic_types_value.bool_value';
info.MatPath{3} = 'basic_types_value.byte_value';
info.MatPath{4} = 'basic_types_value.char_value';
info.MatPath{5} = 'basic_types_value.float32_value';
info.MatPath{6} = 'basic_types_value.float64_value';
info.MatPath{7} = 'basic_types_value.int8_value';
info.MatPath{8} = 'basic_types_value.uint8_value';
info.MatPath{9} = 'basic_types_value.int16_value';
info.MatPath{10} = 'basic_types_value.uint16_value';
info.MatPath{11} = 'basic_types_value.int32_value';
info.MatPath{12} = 'basic_types_value.uint32_value';
info.MatPath{13} = 'basic_types_value.int64_value';
info.MatPath{14} = 'basic_types_value.uint64_value';
