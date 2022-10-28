function [data, info] = deleteMapRequest
%DeleteMap gives an empty data for map_store/DeleteMapRequest

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'map_store/DeleteMapRequest';
[data.MapId, info.MapId] = ros.internal.ros.messages.ros.char('string',0);
info.MessageType = 'map_store/DeleteMapRequest';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,1);
info.MatPath{1} = 'map_id';
