function [data, info] = parameterType
%ParameterType gives an empty data for rcl_interfaces/ParameterType

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'rcl_interfaces/ParameterType';
[data.PARAMETER_NOT_SET, info.PARAMETER_NOT_SET] = ros.internal.ros2.messages.ros2.default_type('uint8',1,0, 0, [NaN]);
[data.PARAMETER_BOOL, info.PARAMETER_BOOL] = ros.internal.ros2.messages.ros2.default_type('uint8',1,0, 1, [NaN]);
[data.PARAMETER_INTEGER, info.PARAMETER_INTEGER] = ros.internal.ros2.messages.ros2.default_type('uint8',1,0, 2, [NaN]);
[data.PARAMETER_DOUBLE, info.PARAMETER_DOUBLE] = ros.internal.ros2.messages.ros2.default_type('uint8',1,0, 3, [NaN]);
[data.PARAMETER_STRING, info.PARAMETER_STRING] = ros.internal.ros2.messages.ros2.default_type('uint8',1,0, 4, [NaN]);
[data.PARAMETER_BYTE_ARRAY, info.PARAMETER_BYTE_ARRAY] = ros.internal.ros2.messages.ros2.default_type('uint8',1,0, 5, [NaN]);
[data.PARAMETER_BOOL_ARRAY, info.PARAMETER_BOOL_ARRAY] = ros.internal.ros2.messages.ros2.default_type('uint8',1,0, 6, [NaN]);
[data.PARAMETER_INTEGER_ARRAY, info.PARAMETER_INTEGER_ARRAY] = ros.internal.ros2.messages.ros2.default_type('uint8',1,0, 7, [NaN]);
[data.PARAMETER_DOUBLE_ARRAY, info.PARAMETER_DOUBLE_ARRAY] = ros.internal.ros2.messages.ros2.default_type('uint8',1,0, 8, [NaN]);
[data.PARAMETER_STRING_ARRAY, info.PARAMETER_STRING_ARRAY] = ros.internal.ros2.messages.ros2.default_type('uint8',1,0, 9, [NaN]);
info.MessageType = 'rcl_interfaces/ParameterType';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,10);
info.MatPath{1} = 'PARAMETER_NOT_SET';
info.MatPath{2} = 'PARAMETER_BOOL';
info.MatPath{3} = 'PARAMETER_INTEGER';
info.MatPath{4} = 'PARAMETER_DOUBLE';
info.MatPath{5} = 'PARAMETER_STRING';
info.MatPath{6} = 'PARAMETER_BYTE_ARRAY';
info.MatPath{7} = 'PARAMETER_BOOL_ARRAY';
info.MatPath{8} = 'PARAMETER_INTEGER_ARRAY';
info.MatPath{9} = 'PARAMETER_DOUBLE_ARRAY';
info.MatPath{10} = 'PARAMETER_STRING_ARRAY';
