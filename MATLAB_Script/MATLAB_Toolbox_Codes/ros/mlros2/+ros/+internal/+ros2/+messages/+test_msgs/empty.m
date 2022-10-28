function [data, info] = empty
%Empty gives an empty data for test_msgs/Empty

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'test_msgs/Empty';
info.MessageType = 'test_msgs/Empty';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,0);
