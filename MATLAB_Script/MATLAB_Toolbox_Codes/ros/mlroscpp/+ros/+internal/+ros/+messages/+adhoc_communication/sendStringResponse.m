function [data, info] = sendStringResponse
%SendString gives an empty data for adhoc_communication/SendStringResponse

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'adhoc_communication/SendStringResponse';
[data.Status, info.Status] = ros.internal.ros.messages.ros.default_type('uint8',1);
info.MessageType = 'adhoc_communication/SendStringResponse';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,1);
info.MatPath{1} = 'status';