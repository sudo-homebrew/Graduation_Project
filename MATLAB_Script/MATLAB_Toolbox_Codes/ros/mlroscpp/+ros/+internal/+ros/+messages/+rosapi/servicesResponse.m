function [data, info] = servicesResponse
%Services gives an empty data for rosapi/ServicesResponse

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'rosapi/ServicesResponse';
[data.Services, info.Services] = ros.internal.ros.messages.ros.char('string',NaN);
info.MessageType = 'rosapi/ServicesResponse';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,1);
info.MatPath{1} = 'services';
