function [data, info] = int64MultiArray
%Int64MultiArray gives an empty data for example_interfaces/Int64MultiArray

% Copyright 2019-2021 The MathWorks, Inc.
data = struct();
data.MessageType = 'example_interfaces/Int64MultiArray';
[data.layout, info.layout] = ros.internal.ros2.messages.example_interfaces.multiArrayLayout;
info.layout.MLdataType = 'struct';
[data.data, info.data] = ros.internal.ros2.messages.ros2.default_type('int64',NaN,0);
info.MessageType = 'example_interfaces/Int64MultiArray';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,7);
info.MatPath{1} = 'layout';
info.MatPath{2} = 'layout.dim';
info.MatPath{3} = 'layout.dim.label';
info.MatPath{4} = 'layout.dim.size';
info.MatPath{5} = 'layout.dim.stride';
info.MatPath{6} = 'layout.data_offset';
info.MatPath{7} = 'data';