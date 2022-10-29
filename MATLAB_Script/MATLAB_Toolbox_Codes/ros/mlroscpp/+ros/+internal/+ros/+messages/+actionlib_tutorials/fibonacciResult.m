function [data, info] = fibonacciResult
%FibonacciResult gives an empty data for actionlib_tutorials/FibonacciResult

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'actionlib_tutorials/FibonacciResult';
[data.Sequence, info.Sequence] = ros.internal.ros.messages.ros.default_type('int32',NaN);
info.MessageType = 'actionlib_tutorials/FibonacciResult';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,1);
info.MatPath{1} = 'sequence';
