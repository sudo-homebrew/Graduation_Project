function [data, info] = addTwoIntsResponse
%AddTwoInts gives an empty data for roseus/AddTwoIntsResponse

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'roseus/AddTwoIntsResponse';
[data.Sum, info.Sum] = ros.internal.ros.messages.ros.default_type('int64',1);
info.MessageType = 'roseus/AddTwoIntsResponse';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,1);
info.MatPath{1} = 'sum';
