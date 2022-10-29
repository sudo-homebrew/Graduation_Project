function [data, info] = listMapsRequest
%ListMaps gives an empty data for map_store/ListMapsRequest

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'map_store/ListMapsRequest';
info.MessageType = 'map_store/ListMapsRequest';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,0);
