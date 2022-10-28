function [data, info] = getMapGoal
%GetMapGoal gives an empty data for nav_msgs/GetMapGoal

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'nav_msgs/GetMapGoal';
info.MessageType = 'nav_msgs/GetMapGoal';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,0);
