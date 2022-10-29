function [data, info] = teleportAbsoluteResponse
%TeleportAbsolute gives an empty data for turtlesim/TeleportAbsoluteResponse

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'turtlesim/TeleportAbsoluteResponse';
info.MessageType = 'turtlesim/TeleportAbsoluteResponse';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,0);
