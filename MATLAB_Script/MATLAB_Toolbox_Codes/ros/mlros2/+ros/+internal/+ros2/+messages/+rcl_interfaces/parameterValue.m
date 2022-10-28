function [data, info] = parameterValue
%ParameterValue gives an empty data for rcl_interfaces/ParameterValue

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'rcl_interfaces/ParameterValue';
[data.type, info.type] = ros.internal.ros2.messages.ros2.default_type('uint8',1,0);
[data.bool_value, info.bool_value] = ros.internal.ros2.messages.ros2.default_type('logical',1,0);
[data.integer_value, info.integer_value] = ros.internal.ros2.messages.ros2.default_type('int64',1,0);
[data.double_value, info.double_value] = ros.internal.ros2.messages.ros2.default_type('double',1,0);
[data.string_value, info.string_value] = ros.internal.ros2.messages.ros2.char('string',1,NaN,0);
[data.byte_array_value, info.byte_array_value] = ros.internal.ros2.messages.ros2.default_type('uint8',NaN,0);
[data.bool_array_value, info.bool_array_value] = ros.internal.ros2.messages.ros2.default_type('logical',NaN,0);
[data.integer_array_value, info.integer_array_value] = ros.internal.ros2.messages.ros2.default_type('int64',NaN,0);
[data.double_array_value, info.double_array_value] = ros.internal.ros2.messages.ros2.default_type('double',NaN,0);
[data.string_array_value, info.string_array_value] = ros.internal.ros2.messages.ros2.char('string',NaN,NaN,0);
info.MessageType = 'rcl_interfaces/ParameterValue';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,10);
info.MatPath{1} = 'type';
info.MatPath{2} = 'bool_value';
info.MatPath{3} = 'integer_value';
info.MatPath{4} = 'double_value';
info.MatPath{5} = 'string_value';
info.MatPath{6} = 'byte_array_value';
info.MatPath{7} = 'bool_array_value';
info.MatPath{8} = 'integer_array_value';
info.MatPath{9} = 'double_array_value';
info.MatPath{10} = 'string_array_value';
