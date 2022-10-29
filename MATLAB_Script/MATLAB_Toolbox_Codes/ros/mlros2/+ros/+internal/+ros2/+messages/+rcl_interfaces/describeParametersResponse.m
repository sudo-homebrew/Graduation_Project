function [data, info] = describeParametersResponse
%DescribeParameters gives an empty data for rcl_interfaces/DescribeParametersResponse

% Copyright 2019-2021 The MathWorks, Inc.
data = struct();
data.MessageType = 'rcl_interfaces/DescribeParametersResponse';
[data.descriptors, info.descriptors] = ros.internal.ros2.messages.rcl_interfaces.parameterDescriptor;
info.descriptors.MLdataType = 'struct';
info.descriptors.MaxLen = NaN;
info.descriptors.MinLen = 0;
info.MessageType = 'rcl_interfaces/DescribeParametersResponse';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,14);
info.MatPath{1} = 'descriptors';
info.MatPath{2} = 'descriptors.name';
info.MatPath{3} = 'descriptors.type';
info.MatPath{4} = 'descriptors.description';
info.MatPath{5} = 'descriptors.additional_constraints';
info.MatPath{6} = 'descriptors.read_only';
info.MatPath{7} = 'descriptors.floating_point_range';
info.MatPath{8} = 'descriptors.floating_point_range.from_value';
info.MatPath{9} = 'descriptors.floating_point_range.to_value';
info.MatPath{10} = 'descriptors.floating_point_range.step';
info.MatPath{11} = 'descriptors.integer_range';
info.MatPath{12} = 'descriptors.integer_range.from_value';
info.MatPath{13} = 'descriptors.integer_range.to_value';
info.MatPath{14} = 'descriptors.integer_range.step';
