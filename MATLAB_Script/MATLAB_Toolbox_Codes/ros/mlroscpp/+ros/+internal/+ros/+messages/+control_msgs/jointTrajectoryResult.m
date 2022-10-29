function [data, info] = jointTrajectoryResult
%JointTrajectoryResult gives an empty data for control_msgs/JointTrajectoryResult

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'control_msgs/JointTrajectoryResult';
info.MessageType = 'control_msgs/JointTrajectoryResult';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,0);
