function [data, info] = parameter
%Parameter gives an empty data for rcl_interfaces/Parameter

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'rcl_interfaces/Parameter';
[data.name, info.name] = ros.internal.ros2.messages.ros2.char('string',1,NaN,0);
[data.value, info.value] = ros.internal.ros2.messages.rcl_interfaces.parameterValue;
info.value.MLdataType = 'struct';
info.MessageType = 'rcl_interfaces/Parameter';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,12);
info.MatPath{1} = 'name';
info.MatPath{2} = 'value';
info.MatPath{3} = 'value.type';
info.MatPath{4} = 'value.bool_value';
info.MatPath{5} = 'value.integer_value';
info.MatPath{6} = 'value.double_value';
info.MatPath{7} = 'value.string_value';
info.MatPath{8} = 'value.byte_array_value';
info.MatPath{9} = 'value.bool_array_value';
info.MatPath{10} = 'value.integer_array_value';
info.MatPath{11} = 'value.double_array_value';
info.MatPath{12} = 'value.string_array_value';
