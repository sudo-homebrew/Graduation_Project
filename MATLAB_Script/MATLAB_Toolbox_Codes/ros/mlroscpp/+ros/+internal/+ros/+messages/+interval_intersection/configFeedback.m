function [data, info] = configFeedback
%ConfigFeedback gives an empty data for interval_intersection/ConfigFeedback

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'interval_intersection/ConfigFeedback';
info.MessageType = 'interval_intersection/ConfigFeedback';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,0);
