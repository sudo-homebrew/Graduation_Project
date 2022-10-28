function [data, info] = ptuResetResult
%PtuResetResult gives an empty data for ptu_control/PtuResetResult

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'ptu_control/PtuResetResult';
info.MessageType = 'ptu_control/PtuResetResult';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,0);
