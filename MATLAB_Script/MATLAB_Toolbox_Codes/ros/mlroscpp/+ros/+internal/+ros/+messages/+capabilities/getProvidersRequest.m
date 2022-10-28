function [data, info] = getProvidersRequest
%GetProviders gives an empty data for capabilities/GetProvidersRequest

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'capabilities/GetProvidersRequest';
[data.Interface, info.Interface] = ros.internal.ros.messages.ros.char('string',0);
[data.IncludeSemantic, info.IncludeSemantic] = ros.internal.ros.messages.ros.default_type('logical',1);
info.MessageType = 'capabilities/GetProvidersRequest';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,2);
info.MatPath{1} = 'interface';
info.MatPath{2} = 'include_semantic';
