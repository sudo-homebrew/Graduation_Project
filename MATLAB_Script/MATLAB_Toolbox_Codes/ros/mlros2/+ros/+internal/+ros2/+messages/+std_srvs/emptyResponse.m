function [data, info] = emptyResponse
%Empty gives an empty data for std_srvs/EmptyResponse

% Copyright 2019-2021 The MathWorks, Inc.
data = struct();
data.MessageType = 'std_srvs/EmptyResponse';
info.MessageType = 'std_srvs/EmptyResponse';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,0);
