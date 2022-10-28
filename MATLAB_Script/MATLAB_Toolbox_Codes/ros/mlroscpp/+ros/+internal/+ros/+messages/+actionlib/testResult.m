function [data, info] = testResult
%TestResult gives an empty data for actionlib/TestResult

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'actionlib/TestResult';
[data.Result, info.Result] = ros.internal.ros.messages.ros.default_type('int32',1);
info.MessageType = 'actionlib/TestResult';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,1);
info.MatPath{1} = 'result';
