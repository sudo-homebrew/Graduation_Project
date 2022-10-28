function [data, info] = resetResponse
%Reset gives an empty data for um6/ResetResponse

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'um6/ResetResponse';
info.MessageType = 'um6/ResetResponse';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,0);
