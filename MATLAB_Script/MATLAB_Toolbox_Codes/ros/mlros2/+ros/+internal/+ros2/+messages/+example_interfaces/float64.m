function [data, info] = float64
%Float64 gives an empty data for example_interfaces/Float64

% Copyright 2019-2021 The MathWorks, Inc.
data = struct();
data.MessageType = 'example_interfaces/Float64';
[data.data, info.data] = ros.internal.ros2.messages.ros2.default_type('double',1,0);
info.MessageType = 'example_interfaces/Float64';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,1);
info.MatPath{1} = 'data';
