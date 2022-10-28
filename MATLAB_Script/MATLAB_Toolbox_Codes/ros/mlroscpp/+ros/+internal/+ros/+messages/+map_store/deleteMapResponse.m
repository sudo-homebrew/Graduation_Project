function [data, info] = deleteMapResponse
%DeleteMap gives an empty data for map_store/DeleteMapResponse

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'map_store/DeleteMapResponse';
info.MessageType = 'map_store/DeleteMapResponse';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,0);
