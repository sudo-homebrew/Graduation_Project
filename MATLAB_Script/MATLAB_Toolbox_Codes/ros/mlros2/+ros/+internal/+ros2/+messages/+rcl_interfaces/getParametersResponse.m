function [data, info] = getParametersResponse
%GetParameters gives an empty data for rcl_interfaces/GetParametersResponse

% Copyright 2019-2021 The MathWorks, Inc.
data = struct();
data.MessageType = 'rcl_interfaces/GetParametersResponse';
[data.values, info.values] = ros.internal.ros2.messages.rcl_interfaces.parameterValue;
info.values.MLdataType = 'struct';
info.values.MaxLen = NaN;
info.values.MinLen = 0;
info.MessageType = 'rcl_interfaces/GetParametersResponse';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,11);
info.MatPath{1} = 'values';
info.MatPath{2} = 'values.type';
info.MatPath{3} = 'values.bool_value';
info.MatPath{4} = 'values.integer_value';
info.MatPath{5} = 'values.double_value';
info.MatPath{6} = 'values.string_value';
info.MatPath{7} = 'values.byte_array_value';
info.MatPath{8} = 'values.bool_array_value';
info.MatPath{9} = 'values.integer_array_value';
info.MatPath{10} = 'values.double_array_value';
info.MatPath{11} = 'values.string_array_value';
