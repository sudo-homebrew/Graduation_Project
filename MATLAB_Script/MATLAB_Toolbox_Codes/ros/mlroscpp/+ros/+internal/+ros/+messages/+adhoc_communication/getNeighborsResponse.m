function [data, info] = getNeighborsResponse
%GetNeighbors gives an empty data for adhoc_communication/GetNeighborsResponse

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'adhoc_communication/GetNeighborsResponse';
[data.Neigbors, info.Neigbors] = ros.internal.ros.messages.ros.char('string',NaN);
info.MessageType = 'adhoc_communication/GetNeighborsResponse';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,1);
info.MatPath{1} = 'neigbors';
