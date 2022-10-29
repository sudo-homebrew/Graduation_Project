function [data, info] = logMapsResponse
%LogMaps gives an empty data for map_merger/LogMapsResponse

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'map_merger/LogMapsResponse';
info.MessageType = 'map_merger/LogMapsResponse';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,0);
