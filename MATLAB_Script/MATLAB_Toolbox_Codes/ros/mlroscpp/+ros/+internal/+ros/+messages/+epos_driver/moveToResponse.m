function [data, info] = moveToResponse
%MoveTo gives an empty data for epos_driver/MoveToResponse

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'epos_driver/MoveToResponse';
info.MessageType = 'epos_driver/MoveToResponse';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,0);
