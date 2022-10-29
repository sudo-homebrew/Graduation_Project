function [data, info] = renameMapRequest
%RenameMap gives an empty data for map_store/RenameMapRequest

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'map_store/RenameMapRequest';
[data.MapId, info.MapId] = ros.internal.ros.messages.ros.char('string',0);
[data.NewName, info.NewName] = ros.internal.ros.messages.ros.char('string',0);
info.MessageType = 'map_store/RenameMapRequest';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,2);
info.MatPath{1} = 'map_id';
info.MatPath{2} = 'new_name';
