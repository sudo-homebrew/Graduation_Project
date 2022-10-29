function [data, info] = emptyRequest
%Empty gives an empty data for std_srvs/EmptyRequest

% Copyright 2019-2021 The MathWorks, Inc.
data = struct();
data.MessageType = 'std_srvs/EmptyRequest';
info.MessageType = 'std_srvs/EmptyRequest';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,0);
