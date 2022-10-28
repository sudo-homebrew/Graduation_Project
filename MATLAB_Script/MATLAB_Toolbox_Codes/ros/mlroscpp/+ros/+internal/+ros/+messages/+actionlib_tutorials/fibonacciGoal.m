function [data, info] = fibonacciGoal
%FibonacciGoal gives an empty data for actionlib_tutorials/FibonacciGoal

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'actionlib_tutorials/FibonacciGoal';
[data.Order, info.Order] = ros.internal.ros.messages.ros.default_type('int32',1);
info.MessageType = 'actionlib_tutorials/FibonacciGoal';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,1);
info.MatPath{1} = 'order';
