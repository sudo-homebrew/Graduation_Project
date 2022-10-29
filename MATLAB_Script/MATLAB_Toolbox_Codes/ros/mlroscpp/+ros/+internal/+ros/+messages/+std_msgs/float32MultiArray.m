function [data, info] = float32MultiArray
%Float32MultiArray gives an empty data for std_msgs/Float32MultiArray

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'std_msgs/Float32MultiArray';
[data.Layout, info.Layout] = ros.internal.ros.messages.std_msgs.multiArrayLayout;
info.Layout.MLdataType = 'struct';
[data.Data, info.Data] = ros.internal.ros.messages.ros.default_type('single',NaN);
info.MessageType = 'std_msgs/Float32MultiArray';
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
