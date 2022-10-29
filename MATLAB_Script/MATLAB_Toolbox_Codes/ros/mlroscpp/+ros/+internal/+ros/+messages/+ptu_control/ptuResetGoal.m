function [data, info] = ptuResetGoal
%PtuResetGoal gives an empty data for ptu_control/PtuResetGoal

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'ptu_control/PtuResetGoal';
info.MessageType = 'ptu_control/PtuResetGoal';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,0);
