function [data, info] = uInt8
%UInt8 gives an empty data for example_interfaces/UInt8

% Copyright 2019-2021 The MathWorks, Inc.
data = struct();
data.MessageType = 'example_interfaces/UInt8';
[data.data, info.data] = ros.internal.ros2.messages.ros2.default_type('uint8',1,0);
info.MessageType = 'example_interfaces/UInt8';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,1);
info.MatPath{1} = 'data';
