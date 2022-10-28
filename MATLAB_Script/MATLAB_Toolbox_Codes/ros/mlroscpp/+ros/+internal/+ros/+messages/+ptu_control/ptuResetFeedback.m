function [data, info] = ptuResetFeedback
%PtuResetFeedback gives an empty data for ptu_control/PtuResetFeedback

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'ptu_control/PtuResetFeedback';
info.MessageType = 'ptu_control/PtuResetFeedback';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,0);
