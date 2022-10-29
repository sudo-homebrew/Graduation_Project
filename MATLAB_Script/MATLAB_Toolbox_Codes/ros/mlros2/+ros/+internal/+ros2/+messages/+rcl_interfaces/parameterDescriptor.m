function [data, info] = parameterDescriptor
%ParameterDescriptor gives an empty data for rcl_interfaces/ParameterDescriptor

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'rcl_interfaces/ParameterDescriptor';
[data.name, info.name] = ros.internal.ros2.messages.ros2.char('string',1,NaN,0);
[data.type, info.type] = ros.internal.ros2.messages.ros2.default_type('uint8',1,0);
[data.description, info.description] = ros.internal.ros2.messages.ros2.char('string',1,NaN,0);
[data.additional_constraints, info.additional_constraints] = ros.internal.ros2.messages.ros2.char('string',1,NaN,0);
[data.read_only, info.read_only] = ros.internal.ros2.messages.ros2.default_type('logical',1,0, NaN, [0]);
[data.floating_point_range, info.floating_point_range] = ros.internal.ros2.messages.rcl_interfaces.floatingPointRange;
info.floating_point_range.MLdataType = 'struct';
[data.integer_range, info.integer_range] = ros.internal.ros2.messages.rcl_interfaces.integerRange;
info.integer_range.MLdataType = 'struct';
val = [];
for i = 1:1
    val = vertcat(data.floating_point_range,val); %#ok<AGROW>
end
data.floating_point_range = val;
val = [];
for i = 1:1
    val = vertcat(data.integer_range,val); %#ok<AGROW>
end
data.integer_range = val;
info.MessageType = 'rcl_interfaces/ParameterDescriptor';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,13);
info.MatPath{1} = 'name';
info.MatPath{2} = 'type';
info.MatPath{3} = 'description';
info.MatPath{4} = 'additional_constraints';
info.MatPath{5} = 'read_only';
info.MatPath{6} = 'floating_point_range';
info.MatPath{7} = 'floating_point_range.from_value';
info.MatPath{8} = 'floating_point_range.to_value';
info.MatPath{9} = 'floating_point_range.step';
info.MatPath{10} = 'integer_range';
info.MatPath{11} = 'integer_range.from_value';
info.MatPath{12} = 'integer_range.to_value';
info.MatPath{13} = 'integer_range.step';
