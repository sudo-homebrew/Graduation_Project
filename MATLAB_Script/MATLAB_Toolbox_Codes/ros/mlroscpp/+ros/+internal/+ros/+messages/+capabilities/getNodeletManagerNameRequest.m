function [data, info] = getNodeletManagerNameRequest
%GetNodeletManagerName gives an empty data for capabilities/GetNodeletManagerNameRequest

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'capabilities/GetNodeletManagerNameRequest';
info.MessageType = 'capabilities/GetNodeletManagerNameRequest';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,0);
