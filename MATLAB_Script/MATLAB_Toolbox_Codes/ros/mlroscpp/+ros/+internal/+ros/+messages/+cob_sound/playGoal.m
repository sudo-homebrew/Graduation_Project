function [data, info] = playGoal
%PlayGoal gives an empty data for cob_sound/PlayGoal

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'cob_sound/PlayGoal';
[data.Filename, info.Filename] = ros.internal.ros.messages.ros.char('string',0);
info.MessageType = 'cob_sound/PlayGoal';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,1);
info.MatPath{1} = 'filename';
