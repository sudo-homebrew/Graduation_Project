function [data, info] = testResult
%TestResult gives an empty data for asmach_tutorials/TestResult

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'asmach_tutorials/TestResult';
info.MessageType = 'asmach_tutorials/TestResult';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,0);
