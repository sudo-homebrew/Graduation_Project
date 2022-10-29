function [data, info] = pose2D
%Pose2D gives an empty data for geometry_msgs/Pose2D

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'geometry_msgs/Pose2D';
[data.X, info.X] = ros.internal.ros.messages.ros.default_type('double',1);
[data.Y, info.Y] = ros.internal.ros.messages.ros.default_type('double',1);
[data.Theta, info.Theta] = ros.internal.ros.messages.ros.default_type('double',1);
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
