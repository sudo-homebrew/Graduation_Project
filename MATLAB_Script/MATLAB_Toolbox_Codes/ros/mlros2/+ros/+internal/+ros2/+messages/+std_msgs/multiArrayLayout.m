function [data, info] = multiArrayLayout
%MultiArrayLayout gives an empty data for std_msgs/MultiArrayLayout

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'std_msgs/MultiArrayLayout';
[data.dim, info.dim] = ros.internal.ros2.messages.std_msgs.multiArrayDimension;
info.dim.MLdataType = 'struct';
info.dim.MaxLen = NaN;
info.dim.MinLen = 0;
[data.data_offset, info.data_offset] = ros.internal.ros2.messages.ros2.default_type('uint32',1,0);
info.MessageType = 'std_msgs/MultiArrayLayout';
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
