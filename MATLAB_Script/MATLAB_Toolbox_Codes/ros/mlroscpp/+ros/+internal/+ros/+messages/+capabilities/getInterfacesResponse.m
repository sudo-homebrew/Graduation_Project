function [data, info] = getInterfacesResponse
%GetInterfaces gives an empty data for capabilities/GetInterfacesResponse

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'capabilities/GetInterfacesResponse';
[data.Interfaces, info.Interfaces] = ros.internal.ros.messages.ros.char('string',NaN);
info.MessageType = 'capabilities/GetInterfacesResponse';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,1);
info.MatPath{1} = 'interfaces';
