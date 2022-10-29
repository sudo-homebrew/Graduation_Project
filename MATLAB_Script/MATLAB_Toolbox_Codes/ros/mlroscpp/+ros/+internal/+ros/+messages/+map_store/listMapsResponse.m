function [data, info] = listMapsResponse
%ListMaps gives an empty data for map_store/ListMapsResponse

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'map_store/ListMapsResponse';
[data.MapList, info.MapList] = ros.internal.ros.messages.map_store.mapListEntry;
info.MapList.MLdataType = 'struct';
info.MapList.MaxLen = NaN;
info.MapList.MinLen = 0;
data.MapList = data.MapList([],1);
info.MessageType = 'map_store/ListMapsResponse';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,5);
info.MatPath{1} = 'map_list';
info.MatPath{2} = 'map_list.name';
info.MatPath{3} = 'map_list.session_id';
info.MatPath{4} = 'map_list.date';
info.MatPath{5} = 'map_list.map_id';
