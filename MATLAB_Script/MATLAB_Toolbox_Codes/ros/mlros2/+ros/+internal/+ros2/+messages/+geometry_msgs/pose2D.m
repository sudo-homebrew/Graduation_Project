function [data, info] = pose2D
%Pose2D gives an empty data for geometry_msgs/Pose2D

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'geometry_msgs/Pose2D';
[data.x, info.x] = ros.internal.ros2.messages.ros2.default_type('double',1,0);
[data.y, info.y] = ros.internal.ros2.messages.ros2.default_type('double',1,0);
[data.theta, info.theta] = ros.internal.ros2.messages.ros2.default_type('double',1,0);
info.MessageType = 'geometry_msgs/Pose2D';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,3);
info.MatPath{1} = 'x';
info.MatPath{2} = 'y';
info.MatPath{3} = 'theta';
