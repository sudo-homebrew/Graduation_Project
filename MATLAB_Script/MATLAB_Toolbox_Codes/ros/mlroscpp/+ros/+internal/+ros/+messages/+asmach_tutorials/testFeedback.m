function [data, info] = testFeedback
%TestFeedback gives an empty data for asmach_tutorials/TestFeedback

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'asmach_tutorials/TestFeedback';
info.MessageType = 'asmach_tutorials/TestFeedback';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,0);
