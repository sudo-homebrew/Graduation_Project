function [data, info] = logMapsRequest
%LogMaps gives an empty data for map_merger/LogMapsRequest

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'map_merger/LogMapsRequest';
[data.Log, info.Log] = ros.internal.ros.messages.ros.default_type('uint16',1);
info.MessageType = 'map_merger/LogMapsRequest';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,1);
info.MatPath{1} = 'log';
