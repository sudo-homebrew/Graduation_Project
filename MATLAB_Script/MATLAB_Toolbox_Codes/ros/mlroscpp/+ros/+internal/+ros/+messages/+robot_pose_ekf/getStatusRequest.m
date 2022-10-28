function [data, info] = getStatusRequest
%GetStatus gives an empty data for robot_pose_ekf/GetStatusRequest

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'robot_pose_ekf/GetStatusRequest';
info.MessageType = 'robot_pose_ekf/GetStatusRequest';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,0);
