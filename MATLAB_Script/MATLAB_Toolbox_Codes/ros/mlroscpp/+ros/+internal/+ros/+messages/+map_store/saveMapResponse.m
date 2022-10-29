function [data, info] = saveMapResponse
%SaveMap gives an empty data for map_store/SaveMapResponse

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'map_store/SaveMapResponse';
info.MessageType = 'map_store/SaveMapResponse';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,0);
