function [data, info] = rawLidarCal
%RawLidarCal gives an empty data for multisense_ros/RawLidarCal

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'multisense_ros/RawLidarCal';
[data.LaserToSpindle, info.LaserToSpindle] = ros.internal.ros.messages.ros.default_type('single',16);
[data.CameraToSpindleFixed, info.CameraToSpindleFixed] = ros.internal.ros.messages.ros.default_type('single',16);
info.MessageType = 'multisense_ros/RawLidarCal';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,2);
info.MatPath{1} = 'laserToSpindle';
info.MatPath{2} = 'cameraToSpindleFixed';
