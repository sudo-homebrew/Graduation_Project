function [data, info] = getParamNamesRequest
%GetParamNames gives an empty data for rosapi/GetParamNamesRequest

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'rosapi/GetParamNamesRequest';
info.MessageType = 'rosapi/GetParamNamesRequest';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,0);
