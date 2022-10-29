function [data, info] = twoIntsResult
%TwoIntsResult gives an empty data for actionlib/TwoIntsResult

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'actionlib/TwoIntsResult';
[data.Sum, info.Sum] = ros.internal.ros.messages.ros.default_type('int64',1);
info.MessageType = 'actionlib/TwoIntsResult';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,1);
info.MatPath{1} = 'sum';
