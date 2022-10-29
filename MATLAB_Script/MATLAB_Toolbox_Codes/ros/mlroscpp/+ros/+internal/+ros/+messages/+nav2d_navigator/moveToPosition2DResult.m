function [data, info] = moveToPosition2DResult
%MoveToPosition2DResult gives an empty data for nav2d_navigator/MoveToPosition2DResult

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'nav2d_navigator/MoveToPosition2DResult';
[data.FinalPose, info.FinalPose] = ros.internal.ros.messages.geometry_msgs.pose2D;
info.FinalPose.MLdataType = 'struct';
[data.FinalDistance, info.FinalDistance] = ros.internal.ros.messages.ros.default_type('single',1);
info.MessageType = 'nav2d_navigator/MoveToPosition2DResult';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,5);
info.MatPath{1} = 'final_pose';
info.MatPath{2} = 'final_pose.x';
info.MatPath{3} = 'final_pose.y';
info.MatPath{4} = 'final_pose.theta';
info.MatPath{5} = 'final_distance';
