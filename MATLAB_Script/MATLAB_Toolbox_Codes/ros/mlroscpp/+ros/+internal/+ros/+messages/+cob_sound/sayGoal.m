function [data, info] = sayGoal
%SayGoal gives an empty data for cob_sound/SayGoal

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'cob_sound/SayGoal';
[data.Text, info.Text] = ros.internal.ros.messages.ros.char('string',0);
info.MessageType = 'cob_sound/SayGoal';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,1);
info.MatPath{1} = 'text';
