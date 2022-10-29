function [data, info] = testGoal
%TestGoal gives an empty data for asmach_tutorials/TestGoal

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'asmach_tutorials/TestGoal';
[data.Goal, info.Goal] = ros.internal.ros.messages.ros.default_type('double',1);
info.MessageType = 'asmach_tutorials/TestGoal';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,1);
info.MatPath{1} = 'goal';
