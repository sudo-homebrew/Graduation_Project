function [data, info] = addTwoIntsResponse
%AddTwoInts gives an empty data for example_interfaces/AddTwoIntsResponse

% Copyright 2019-2021 The MathWorks, Inc.
data = struct();
data.MessageType = 'example_interfaces/AddTwoIntsResponse';
[data.sum, info.sum] = ros.internal.ros2.messages.ros2.default_type('int64',1,0);
info.MessageType = 'example_interfaces/AddTwoIntsResponse';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,1);
info.MatPath{1} = 'sum';
