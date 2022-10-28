function [data, info] = teleportRelativeResponse
%TeleportRelative gives an empty data for turtlesim/TeleportRelativeResponse

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'turtlesim/TeleportRelativeResponse';
info.MessageType = 'turtlesim/TeleportRelativeResponse';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,0);
