function [data, info] = moveBaseGoal
%MoveBaseGoal gives an empty data for move_base_msgs/MoveBaseGoal

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'move_base_msgs/MoveBaseGoal';
[data.TargetPose, info.TargetPose] = ros.internal.ros.messages.geometry_msgs.poseStamped;
info.TargetPose.MLdataType = 'struct';
info.MessageType = 'move_base_msgs/MoveBaseGoal';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,17);
info.MatPath{1} = 'target_pose';
info.MatPath{2} = 'target_pose.header';
info.MatPath{3} = 'target_pose.header.seq';
info.MatPath{4} = 'target_pose.header.stamp';
info.MatPath{5} = 'target_pose.header.stamp.sec';
info.MatPath{6} = 'target_pose.header.stamp.nsec';
info.MatPath{7} = 'target_pose.header.frame_id';
info.MatPath{8} = 'target_pose.pose';
info.MatPath{9} = 'target_pose.pose.position';
info.MatPath{10} = 'target_pose.pose.position.x';
info.MatPath{11} = 'target_pose.pose.position.y';
info.MatPath{12} = 'target_pose.pose.position.z';
info.MatPath{13} = 'target_pose.pose.orientation';
info.MatPath{14} = 'target_pose.pose.orientation.x';
info.MatPath{15} = 'target_pose.pose.orientation.y';
info.MatPath{16} = 'target_pose.pose.orientation.z';
info.MatPath{17} = 'target_pose.pose.orientation.w';
