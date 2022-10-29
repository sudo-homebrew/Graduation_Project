function [data, info] = detectObjectsFeedback
%DetectObjectsFeedback gives an empty data for cob_object_detection_msgs/DetectObjectsFeedback

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'cob_object_detection_msgs/DetectObjectsFeedback';
[data.PercentComplete, info.PercentComplete] = ros.internal.ros.messages.ros.default_type('single',1);
info.MessageType = 'cob_object_detection_msgs/DetectObjectsFeedback';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,1);
info.MatPath{1} = 'percent_complete';
