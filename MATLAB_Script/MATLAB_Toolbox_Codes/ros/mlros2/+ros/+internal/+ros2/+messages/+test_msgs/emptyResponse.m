function [data, info] = emptyResponse
%Empty gives an empty data for test_msgs/EmptyResponse

% Copyright 2019-2021 The MathWorks, Inc.
data = struct();
data.MessageType = 'test_msgs/EmptyResponse';
info.MessageType = 'test_msgs/EmptyResponse';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,0);
