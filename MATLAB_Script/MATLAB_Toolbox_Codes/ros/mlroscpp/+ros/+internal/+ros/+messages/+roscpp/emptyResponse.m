function [data, info] = emptyResponse
%Empty gives an empty data for roscpp/EmptyResponse

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'roscpp/EmptyResponse';
info.MessageType = 'roscpp/EmptyResponse';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,0);
