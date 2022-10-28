function [data, info] = getTimeRequest
%GetTime gives an empty data for rosapi/GetTimeRequest

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'rosapi/GetTimeRequest';
info.MessageType = 'rosapi/GetTimeRequest';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,0);
