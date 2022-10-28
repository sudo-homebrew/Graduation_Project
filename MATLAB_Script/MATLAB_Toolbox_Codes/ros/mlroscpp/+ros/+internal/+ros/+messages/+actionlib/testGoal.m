function [data, info] = testGoal
%TestGoal gives an empty data for actionlib/TestGoal

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'actionlib/TestGoal';
[data.Goal, info.Goal] = ros.internal.ros.messages.ros.default_type('int32',1);
info.MessageType = 'actionlib/TestGoal';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,1);
info.MatPath{1} = 'goal';
