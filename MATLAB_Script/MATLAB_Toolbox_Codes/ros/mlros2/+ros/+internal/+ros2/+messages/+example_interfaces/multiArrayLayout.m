function [data, info] = multiArrayLayout
%MultiArrayLayout gives an empty data for example_interfaces/MultiArrayLayout

% Copyright 2019-2021 The MathWorks, Inc.
data = struct();
data.MessageType = 'example_interfaces/MultiArrayLayout';
[data.dim, info.dim] = ros.internal.ros2.messages.example_interfaces.multiArrayDimension;
info.dim.MLdataType = 'struct';
info.dim.MaxLen = NaN;
info.dim.MinLen = 0;
[data.data_offset, info.data_offset] = ros.internal.ros2.messages.ros2.default_type('uint32',1,0);
info.MessageType = 'example_interfaces/MultiArrayLayout';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,5);
info.MatPath{1} = 'dim';
info.MatPath{2} = 'dim.label';
info.MatPath{3} = 'dim.size';
info.MatPath{4} = 'dim.stride';
info.MatPath{5} = 'data_offset';
