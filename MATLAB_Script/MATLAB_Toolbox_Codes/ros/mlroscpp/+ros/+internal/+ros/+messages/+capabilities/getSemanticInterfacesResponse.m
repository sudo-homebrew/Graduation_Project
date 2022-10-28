function [data, info] = getSemanticInterfacesResponse
%GetSemanticInterfaces gives an empty data for capabilities/GetSemanticInterfacesResponse

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'capabilities/GetSemanticInterfacesResponse';
[data.SemanticInterfaces, info.SemanticInterfaces] = ros.internal.ros.messages.ros.char('string',NaN);
info.MessageType = 'capabilities/GetSemanticInterfacesResponse';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,1);
info.MatPath{1} = 'semantic_interfaces';
