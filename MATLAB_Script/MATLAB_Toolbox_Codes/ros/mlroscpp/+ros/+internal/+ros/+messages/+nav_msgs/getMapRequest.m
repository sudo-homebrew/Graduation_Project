function [data, info] = getMapRequest
%GetMap gives an empty data for nav_msgs/GetMapRequest

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'nav_msgs/GetMapRequest';
info.MessageType = 'nav_msgs/GetMapRequest';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,0);
