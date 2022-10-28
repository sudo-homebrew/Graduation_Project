function [data, info] = getRobotTrajectoryRequest
%GetRobotTrajectory gives an empty data for hector_nav_msgs/GetRobotTrajectoryRequest

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'hector_nav_msgs/GetRobotTrajectoryRequest';
info.MessageType = 'hector_nav_msgs/GetRobotTrajectoryRequest';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,0);
