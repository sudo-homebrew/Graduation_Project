function [data, info] = shutDownResponse
%ShutDown gives an empty data for adhoc_communication/ShutDownResponse

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'adhoc_communication/ShutDownResponse';
info.MessageType = 'adhoc_communication/ShutDownResponse';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,0);
