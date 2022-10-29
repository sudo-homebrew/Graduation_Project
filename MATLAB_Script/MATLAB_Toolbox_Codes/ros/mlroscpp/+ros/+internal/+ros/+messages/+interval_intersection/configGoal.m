function [data, info] = configGoal
%ConfigGoal gives an empty data for interval_intersection/ConfigGoal

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'interval_intersection/ConfigGoal';
[data.Topics, info.Topics] = ros.internal.ros.messages.ros.char('string',NaN);
info.MessageType = 'interval_intersection/ConfigGoal';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,1);
info.MatPath{1} = 'topics';
