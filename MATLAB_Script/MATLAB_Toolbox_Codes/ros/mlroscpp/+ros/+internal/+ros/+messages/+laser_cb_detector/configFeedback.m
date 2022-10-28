function [data, info] = configFeedback
%ConfigFeedback gives an empty data for laser_cb_detector/ConfigFeedback

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'laser_cb_detector/ConfigFeedback';
info.MessageType = 'laser_cb_detector/ConfigFeedback';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,0);
