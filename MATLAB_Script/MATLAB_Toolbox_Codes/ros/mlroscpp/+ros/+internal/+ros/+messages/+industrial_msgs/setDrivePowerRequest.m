function [data, info] = setDrivePowerRequest
%SetDrivePower gives an empty data for industrial_msgs/SetDrivePowerRequest

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'industrial_msgs/SetDrivePowerRequest';
[data.DrivePower, info.DrivePower] = ros.internal.ros.messages.ros.default_type('logical',1);
info.MessageType = 'industrial_msgs/SetDrivePowerRequest';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,1);
info.MatPath{1} = 'drive_power';
