function [data, info] = multiArrayDimension
%MultiArrayDimension gives an empty data for std_msgs/MultiArrayDimension

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'std_msgs/MultiArrayDimension';
[data.Label, info.Label] = ros.internal.ros.messages.ros.char('string',0);
[data.Size, info.Size] = ros.internal.ros.messages.ros.default_type('uint32',1);
[data.Stride, info.Stride] = ros.internal.ros.messages.ros.default_type('uint32',1);
info.MessageType = 'std_msgs/MultiArrayDimension';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,3);
info.MatPath{1} = 'label';
info.MatPath{2} = 'size';
info.MatPath{3} = 'stride';
