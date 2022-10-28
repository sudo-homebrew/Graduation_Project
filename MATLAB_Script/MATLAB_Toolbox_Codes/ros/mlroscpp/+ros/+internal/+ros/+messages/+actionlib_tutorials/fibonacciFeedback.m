function [data, info] = fibonacciFeedback
%FibonacciFeedback gives an empty data for actionlib_tutorials/FibonacciFeedback

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'actionlib_tutorials/FibonacciFeedback';
[data.Sequence, info.Sequence] = ros.internal.ros.messages.ros.default_type('int32',NaN);
info.MessageType = 'actionlib_tutorials/FibonacciFeedback';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,1);
info.MatPath{1} = 'sequence';
