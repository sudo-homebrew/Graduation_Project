function [data, info] = robotPickupReleasePointResponse
%RobotPickupReleasePoint gives an empty data for jsk_pcl_ros/RobotPickupReleasePointResponse

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'jsk_pcl_ros/RobotPickupReleasePointResponse';
[data.Success, info.Success] = ros.internal.ros.messages.ros.default_type('logical',1);
info.MessageType = 'jsk_pcl_ros/RobotPickupReleasePointResponse';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,1);
info.MatPath{1} = 'success';
