function [data, info] = localizeFeedback
%LocalizeFeedback gives an empty data for nav2d_navigator/LocalizeFeedback

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'nav2d_navigator/LocalizeFeedback';
info.MessageType = 'nav2d_navigator/LocalizeFeedback';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,0);
