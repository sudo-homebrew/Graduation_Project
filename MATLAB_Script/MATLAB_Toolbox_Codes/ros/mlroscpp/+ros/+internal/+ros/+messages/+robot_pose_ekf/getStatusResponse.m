function [data, info] = getStatusResponse
%GetStatus gives an empty data for robot_pose_ekf/GetStatusResponse

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'robot_pose_ekf/GetStatusResponse';
[data.Status, info.Status] = ros.internal.ros.messages.ros.char('string',0);
info.MessageType = 'robot_pose_ekf/GetStatusResponse';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,1);
info.MatPath{1} = 'status';
