function [data, info] = pose2DArray
%Pose2DArray gives an empty data for robotnik_msgs/Pose2DArray

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'robotnik_msgs/Pose2DArray';
[data.Poses, info.Poses] = ros.internal.ros.messages.geometry_msgs.pose2D;
info.Poses.MLdataType = 'struct';
info.Poses.MaxLen = NaN;
info.Poses.MinLen = 0;
data.Poses = data.Poses([],1);
info.MessageType = 'robotnik_msgs/Pose2DArray';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,4);
info.MatPath{1} = 'poses';
info.MatPath{2} = 'poses.x';
info.MatPath{3} = 'poses.y';
info.MatPath{4} = 'poses.theta';
