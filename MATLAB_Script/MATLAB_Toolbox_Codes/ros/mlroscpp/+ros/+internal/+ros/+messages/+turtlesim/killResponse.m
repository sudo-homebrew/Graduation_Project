function [data, info] = killResponse
%Kill gives an empty data for turtlesim/KillResponse

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'turtlesim/KillResponse';
info.MessageType = 'turtlesim/KillResponse';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,0);
