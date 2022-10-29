function [data, info] = lookAroundGroundFeedback
%LookAroundGroundFeedback gives an empty data for jsk_footstep_controller/LookAroundGroundFeedback

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'jsk_footstep_controller/LookAroundGroundFeedback';
info.MessageType = 'jsk_footstep_controller/LookAroundGroundFeedback';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,0);
