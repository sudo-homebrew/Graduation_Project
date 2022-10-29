function [data, info] = getSemanticInterfacesRequest
%GetSemanticInterfaces gives an empty data for capabilities/GetSemanticInterfacesRequest

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'capabilities/GetSemanticInterfacesRequest';
[data.Interface, info.Interface] = ros.internal.ros.messages.ros.char('string',0);
info.MessageType = 'capabilities/GetSemanticInterfacesRequest';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,1);
info.MatPath{1} = 'interface';
