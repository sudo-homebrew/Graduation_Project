function [data, info] = publishMapResponse
%PublishMap gives an empty data for map_store/PublishMapResponse

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'map_store/PublishMapResponse';
info.MessageType = 'map_store/PublishMapResponse';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,0);
