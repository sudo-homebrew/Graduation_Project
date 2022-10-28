function [data, info] = serviceProvidersResponse
%ServiceProviders gives an empty data for rosapi/ServiceProvidersResponse

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'rosapi/ServiceProvidersResponse';
[data.Providers, info.Providers] = ros.internal.ros.messages.ros.char('string',NaN);
info.MessageType = 'rosapi/ServiceProvidersResponse';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,1);
info.MatPath{1} = 'providers';
