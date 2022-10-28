function [data, info] = acquireObjectImageFeedback
%AcquireObjectImageFeedback gives an empty data for cob_object_detection_msgs/AcquireObjectImageFeedback

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'cob_object_detection_msgs/AcquireObjectImageFeedback';
[data.PercentComplete, info.PercentComplete] = ros.internal.ros.messages.ros.default_type('single',1);
info.MessageType = 'cob_object_detection_msgs/AcquireObjectImageFeedback';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,1);
info.MatPath{1} = 'percent_complete';
