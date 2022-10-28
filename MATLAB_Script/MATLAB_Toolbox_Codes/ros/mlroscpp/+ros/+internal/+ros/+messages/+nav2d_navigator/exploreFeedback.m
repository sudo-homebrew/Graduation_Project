function [data, info] = exploreFeedback
%ExploreFeedback gives an empty data for nav2d_navigator/ExploreFeedback

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'nav2d_navigator/ExploreFeedback';
[data.RobotPose, info.RobotPose] = ros.internal.ros.messages.geometry_msgs.pose2D;
info.RobotPose.MLdataType = 'struct';
[data.TargetPose, info.TargetPose] = ros.internal.ros.messages.geometry_msgs.pose2D;
info.TargetPose.MLdataType = 'struct';
[data.Distance, info.Distance] = ros.internal.ros.messages.ros.default_type('single',1);
info.MessageType = 'nav2d_navigator/ExploreFeedback';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,9);
info.MatPath{1} = 'robot_pose';
info.MatPath{2} = 'robot_pose.x';
info.MatPath{3} = 'robot_pose.y';
info.MatPath{4} = 'robot_pose.theta';
info.MatPath{5} = 'target_pose';
info.MatPath{6} = 'target_pose.x';
info.MatPath{7} = 'target_pose.y';
info.MatPath{8} = 'target_pose.theta';
info.MatPath{9} = 'distance';
