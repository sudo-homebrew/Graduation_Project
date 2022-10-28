function [data, info] = loadMapRequest
%LoadMap gives an empty data for nav_msgs/LoadMapRequest

% Copyright 2019-2020 The MathWorks, Inc.
%#codegen
data = struct();
data.MessageType = 'nav_msgs/LoadMapRequest';
[data.MapUrl, info.MapUrl] = ros.internal.ros.messages.ros.char('string',0);
info.MessageType = 'nav_msgs/LoadMapRequest';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,1);
info.MatPath{1} = 'map_url';
