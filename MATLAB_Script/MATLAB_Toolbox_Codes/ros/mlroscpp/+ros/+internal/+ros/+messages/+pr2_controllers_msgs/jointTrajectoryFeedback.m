function [data, info] = jointTrajectoryFeedback
%JointTrajectoryFeedback gives an empty data for pr2_controllers_msgs/JointTrajectoryFeedback

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'pr2_controllers_msgs/JointTrajectoryFeedback';
info.MessageType = 'pr2_controllers_msgs/JointTrajectoryFeedback';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,0);
