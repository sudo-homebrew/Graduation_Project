function [data, info] = saveMapResponse
%SaveMap gives an empty data for map_msgs/SaveMapResponse

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'map_msgs/SaveMapResponse';
info.MessageType = 'map_msgs/SaveMapResponse';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,0);
