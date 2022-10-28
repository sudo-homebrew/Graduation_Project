function [data, info] = listNodesRequest
%ListNodes gives an empty data for composition_interfaces/ListNodesRequest

% Copyright 2019-2021 The MathWorks, Inc.
data = struct();
data.MessageType = 'composition_interfaces/ListNodesRequest';
info.MessageType = 'composition_interfaces/ListNodesRequest';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,0);
