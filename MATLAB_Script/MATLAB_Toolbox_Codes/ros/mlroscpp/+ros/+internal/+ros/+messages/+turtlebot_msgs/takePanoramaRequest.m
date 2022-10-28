function [data, info] = takePanoramaRequest
%TakePanorama gives an empty data for turtlebot_msgs/TakePanoramaRequest

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'turtlebot_msgs/TakePanoramaRequest';
[data.Mode, info.Mode] = ros.internal.ros.messages.ros.default_type('uint8',1);
[data.SNAPANDROTATE, info.SNAPANDROTATE] = ros.internal.ros.messages.ros.default_type('uint8',1, 0);
[data.CONTINUOUS, info.CONTINUOUS] = ros.internal.ros.messages.ros.default_type('uint8',1, 1);
[data.STOP, info.STOP] = ros.internal.ros.messages.ros.default_type('uint8',1, 2);
[data.PanoAngle, info.PanoAngle] = ros.internal.ros.messages.ros.default_type('single',1);
[data.SnapInterval, info.SnapInterval] = ros.internal.ros.messages.ros.default_type('single',1);
[data.RotVel, info.RotVel] = ros.internal.ros.messages.ros.default_type('single',1);
info.MessageType = 'turtlebot_msgs/TakePanoramaRequest';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,7);
info.MatPath{1} = 'mode';
info.MatPath{2} = 'SNAPANDROTATE';
info.MatPath{3} = 'CONTINUOUS';
info.MatPath{4} = 'STOP';
info.MatPath{5} = 'pano_angle';
info.MatPath{6} = 'snap_interval';
info.MatPath{7} = 'rot_vel';
