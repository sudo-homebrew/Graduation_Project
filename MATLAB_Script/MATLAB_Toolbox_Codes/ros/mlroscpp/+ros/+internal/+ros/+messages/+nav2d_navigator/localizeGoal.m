function [data, info] = localizeGoal
%LocalizeGoal gives an empty data for nav2d_navigator/LocalizeGoal

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'nav2d_navigator/LocalizeGoal';
[data.Velocity, info.Velocity] = ros.internal.ros.messages.ros.default_type('single',1);
info.MessageType = 'nav2d_navigator/LocalizeGoal';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,1);
info.MatPath{1} = 'velocity';
