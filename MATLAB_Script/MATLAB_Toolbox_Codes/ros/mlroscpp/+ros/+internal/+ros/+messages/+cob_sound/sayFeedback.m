function [data, info] = sayFeedback
%SayFeedback gives an empty data for cob_sound/SayFeedback

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'cob_sound/SayFeedback';
info.MessageType = 'cob_sound/SayFeedback';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,0);
