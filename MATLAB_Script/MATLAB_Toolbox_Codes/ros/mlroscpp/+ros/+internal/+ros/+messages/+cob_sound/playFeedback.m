function [data, info] = playFeedback
%PlayFeedback gives an empty data for cob_sound/PlayFeedback

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'cob_sound/PlayFeedback';
[data.Position, info.Position] = ros.internal.ros.messages.ros.default_type('single',1);
[data.Time, info.Time] = ros.internal.ros.messages.ros.default_type('int64',1);
info.MessageType = 'cob_sound/PlayFeedback';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,2);
info.MatPath{1} = 'position';
info.MatPath{2} = 'time';
