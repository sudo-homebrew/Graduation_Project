function [data, info] = testEmptyRequest
%TestEmpty gives an empty data for rosbridge_library/TestEmptyRequest

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'rosbridge_library/TestEmptyRequest';
info.MessageType = 'rosbridge_library/TestEmptyRequest';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,0);