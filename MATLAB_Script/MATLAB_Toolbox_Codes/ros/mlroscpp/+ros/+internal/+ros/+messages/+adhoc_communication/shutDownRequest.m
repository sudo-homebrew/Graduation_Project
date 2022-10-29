function [data, info] = shutDownRequest
%ShutDown gives an empty data for adhoc_communication/ShutDownRequest

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'adhoc_communication/ShutDownRequest';
info.MessageType = 'adhoc_communication/ShutDownRequest';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,0);
