function [data, info] = pointHeadFeedback
%PointHeadFeedback gives an empty data for control_msgs/PointHeadFeedback

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'control_msgs/PointHeadFeedback';
[data.PointingAngleError, info.PointingAngleError] = ros.internal.ros.messages.ros.default_type('double',1);
info.MessageType = 'control_msgs/PointHeadFeedback';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,1);
info.MatPath{1} = 'pointing_angle_error';
