function [data, info] = serviceHostResponse
%ServiceHost gives an empty data for rosapi/ServiceHostResponse

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'rosapi/ServiceHostResponse';
[data.Host, info.Host] = ros.internal.ros.messages.ros.char('string',0);
info.MessageType = 'rosapi/ServiceHostResponse';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,1);
info.MatPath{1} = 'host';
