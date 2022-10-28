function [data, info] = setParamResponse
%SetParam gives an empty data for rosapi/SetParamResponse

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'rosapi/SetParamResponse';
info.MessageType = 'rosapi/SetParamResponse';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,0);
