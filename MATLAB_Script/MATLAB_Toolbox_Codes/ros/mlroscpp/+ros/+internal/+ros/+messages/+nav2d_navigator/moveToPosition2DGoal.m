function [data, info] = moveToPosition2DGoal
%MoveToPosition2DGoal gives an empty data for nav2d_navigator/MoveToPosition2DGoal

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'nav2d_navigator/MoveToPosition2DGoal';
[data.Header, info.Header] = ros.internal.ros.messages.std_msgs.header;
info.Header.MLdataType = 'struct';
[data.TargetPose, info.TargetPose] = ros.internal.ros.messages.geometry_msgs.pose2D;
info.TargetPose.MLdataType = 'struct';
[data.TargetDistance, info.TargetDistance] = ros.internal.ros.messages.ros.default_type('single',1);
[data.TargetAngle, info.TargetAngle] = ros.internal.ros.messages.ros.default_type('single',1);
info.MessageType = 'nav2d_navigator/MoveToPosition2DGoal';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,12);
info.MatPath{1} = 'header';
info.MatPath{2} = 'header.seq';
info.MatPath{3} = 'header.stamp';
info.MatPath{4} = 'header.stamp.sec';
info.MatPath{5} = 'header.stamp.nsec';
info.MatPath{6} = 'header.frame_id';
info.MatPath{7} = 'target_pose';
info.MatPath{8} = 'target_pose.x';
info.MatPath{9} = 'target_pose.y';
info.MatPath{10} = 'target_pose.theta';
info.MatPath{11} = 'target_distance';
info.MatPath{12} = 'target_angle';
