function [data, info] = addTwoIntsRequest
%AddTwoInts gives an empty data for example_interfaces/AddTwoIntsRequest

% Copyright 2019-2021 The MathWorks, Inc.
data = struct();
data.MessageType = 'example_interfaces/AddTwoIntsRequest';
[data.a, info.a] = ros.internal.ros2.messages.ros2.default_type('int64',1,0);
[data.b, info.b] = ros.internal.ros2.messages.ros2.default_type('int64',1,0);
info.MessageType = 'example_interfaces/AddTwoIntsRequest';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,2);
info.MatPath{1} = 'a';
info.MatPath{2} = 'b';
