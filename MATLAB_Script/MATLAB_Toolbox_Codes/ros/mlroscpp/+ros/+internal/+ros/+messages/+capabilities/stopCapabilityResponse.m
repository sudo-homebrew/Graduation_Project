function [data, info] = stopCapabilityResponse
%StopCapability gives an empty data for capabilities/StopCapabilityResponse

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'capabilities/StopCapabilityResponse';
[data.Successful, info.Successful] = ros.internal.ros.messages.ros.default_type('logical',1);
info.MessageType = 'capabilities/StopCapabilityResponse';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,1);
info.MatPath{1} = 'successful';
