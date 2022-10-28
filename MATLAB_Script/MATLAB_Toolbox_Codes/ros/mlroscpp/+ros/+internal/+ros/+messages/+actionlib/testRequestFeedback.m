function [data, info] = testRequestFeedback
%TestRequestFeedback gives an empty data for actionlib/TestRequestFeedback

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'actionlib/TestRequestFeedback';
info.MessageType = 'actionlib/TestRequestFeedback';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,0);
