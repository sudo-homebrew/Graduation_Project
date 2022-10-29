function [data, info] = emptyRequest
%Empty gives an empty data for roscpp/EmptyRequest

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'roscpp/EmptyRequest';
info.MessageType = 'roscpp/EmptyRequest';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,0);
