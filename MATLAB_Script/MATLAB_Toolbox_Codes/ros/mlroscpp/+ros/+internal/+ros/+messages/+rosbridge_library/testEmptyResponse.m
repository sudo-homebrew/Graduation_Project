function [data, info] = testEmptyResponse
%TestEmpty gives an empty data for rosbridge_library/TestEmptyResponse

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'rosbridge_library/TestEmptyResponse';
info.MessageType = 'rosbridge_library/TestEmptyResponse';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,0);
