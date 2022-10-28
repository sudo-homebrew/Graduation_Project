function [data, info] = mapListEntry
%MapListEntry gives an empty data for map_store/MapListEntry

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'map_store/MapListEntry';
[data.Name, info.Name] = ros.internal.ros.messages.ros.char('string',0);
[data.SessionId, info.SessionId] = ros.internal.ros.messages.ros.char('string',0);
[data.Date, info.Date] = ros.internal.ros.messages.ros.default_type('int64',1);
[data.MapId, info.MapId] = ros.internal.ros.messages.ros.char('string',0);
info.MessageType = 'map_store/MapListEntry';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,4);
info.MatPath{1} = 'name';
info.MatPath{2} = 'session_id';
info.MatPath{3} = 'date';
info.MatPath{4} = 'map_id';
