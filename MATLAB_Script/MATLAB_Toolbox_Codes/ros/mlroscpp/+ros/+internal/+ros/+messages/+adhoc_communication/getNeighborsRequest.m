function [data, info] = getNeighborsRequest
%GetNeighbors gives an empty data for adhoc_communication/GetNeighborsRequest

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'adhoc_communication/GetNeighborsRequest';
info.MessageType = 'adhoc_communication/GetNeighborsRequest';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,0);
