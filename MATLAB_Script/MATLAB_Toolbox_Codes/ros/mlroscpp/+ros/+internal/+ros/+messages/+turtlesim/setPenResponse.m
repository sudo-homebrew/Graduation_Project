function [data, info] = setPenResponse
%SetPen gives an empty data for turtlesim/SetPenResponse

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'turtlesim/SetPenResponse';
info.MessageType = 'turtlesim/SetPenResponse';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,0);
