function [data, info] = emptyStringRequest
%EmptyString gives an empty data for rocon_std_msgs/EmptyStringRequest

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'rocon_std_msgs/EmptyStringRequest';
info.MessageType = 'rocon_std_msgs/EmptyStringRequest';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,0);