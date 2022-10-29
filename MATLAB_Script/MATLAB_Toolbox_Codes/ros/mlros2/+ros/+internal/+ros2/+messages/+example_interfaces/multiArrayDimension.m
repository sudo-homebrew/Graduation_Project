function [data, info] = multiArrayDimension
%MultiArrayDimension gives an empty data for example_interfaces/MultiArrayDimension

% Copyright 2019-2021 The MathWorks, Inc.
data = struct();
data.MessageType = 'example_interfaces/MultiArrayDimension';
[data.label, info.label] = ros.internal.ros2.messages.ros2.char('string',1,NaN,0);
[data.size, info.size] = ros.internal.ros2.messages.ros2.default_type('uint32',1,0);
[data.stride, info.stride] = ros.internal.ros2.messages.ros2.default_type('uint32',1,0);
info.MessageType = 'example_interfaces/MultiArrayDimension';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,3);
info.MatPath{1} = 'label';
info.MatPath{2} = 'size';
info.MatPath{3} = 'stride';
