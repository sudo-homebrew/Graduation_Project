function [data, info] = configFeedback
%ConfigFeedback gives an empty data for image_cb_detector/ConfigFeedback

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'image_cb_detector/ConfigFeedback';
info.MessageType = 'image_cb_detector/ConfigFeedback';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,0);
