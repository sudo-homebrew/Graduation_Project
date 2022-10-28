function [data, info] = setParametersAtomicallyRequest
%SetParametersAtomically gives an empty data for rcl_interfaces/SetParametersAtomicallyRequest

% Copyright 2019-2021 The MathWorks, Inc.
data = struct();
data.MessageType = 'rcl_interfaces/SetParametersAtomicallyRequest';
[data.parameters, info.parameters] = ros.internal.ros2.messages.rcl_interfaces.parameter;
info.parameters.MLdataType = 'struct';
info.parameters.MaxLen = NaN;
info.parameters.MinLen = 0;
info.MessageType = 'rcl_interfaces/SetParametersAtomicallyRequest';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,13);
info.MatPath{1} = 'parameters';
info.MatPath{2} = 'parameters.name';
info.MatPath{3} = 'parameters.value';
info.MatPath{4} = 'parameters.value.type';
info.MatPath{5} = 'parameters.value.bool_value';
info.MatPath{6} = 'parameters.value.integer_value';
info.MatPath{7} = 'parameters.value.double_value';
info.MatPath{8} = 'parameters.value.string_value';
info.MatPath{9} = 'parameters.value.byte_array_value';
info.MatPath{10} = 'parameters.value.bool_array_value';
info.MatPath{11} = 'parameters.value.integer_array_value';
info.MatPath{12} = 'parameters.value.double_array_value';
info.MatPath{13} = 'parameters.value.string_array_value';
