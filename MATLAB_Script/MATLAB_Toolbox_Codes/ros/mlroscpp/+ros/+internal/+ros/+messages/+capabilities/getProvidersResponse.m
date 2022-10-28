function [data, info] = getProvidersResponse
%GetProviders gives an empty data for capabilities/GetProvidersResponse

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'capabilities/GetProvidersResponse';
[data.Providers, info.Providers] = ros.internal.ros.messages.ros.char('string',NaN);
[data.DefaultProvider, info.DefaultProvider] = ros.internal.ros.messages.ros.char('string',0);
info.MessageType = 'capabilities/GetProvidersResponse';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,2);
info.MatPath{1} = 'providers';
info.MatPath{2} = 'default_provider';
