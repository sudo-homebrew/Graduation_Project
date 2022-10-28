function [data, info] = getFirstMapGoal
%GetFirstMapGoal gives an empty data for nav2d_navigator/GetFirstMapGoal

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'nav2d_navigator/GetFirstMapGoal';
info.MessageType = 'nav2d_navigator/GetFirstMapGoal';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,0);
