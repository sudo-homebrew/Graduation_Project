function [data, info] = multiArrayLayout
%MultiArrayLayout gives an empty data for std_msgs/MultiArrayLayout

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'std_msgs/MultiArrayLayout';
[data.Dim, info.Dim] = ros.internal.ros.messages.std_msgs.multiArrayDimension;
info.Dim.MLdataType = 'struct';
info.Dim.MaxLen = NaN;
info.Dim.MinLen = 0;
data.Dim = data.Dim([],1);
[data.DataOffset, info.DataOffset] = ros.internal.ros.messages.ros.default_type('uint32',1);
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
