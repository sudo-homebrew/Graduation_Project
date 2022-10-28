function [data, info] = recoveryStatus
%RecoveryStatus gives an empty data for move_base_msgs/RecoveryStatus

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'move_base_msgs/RecoveryStatus';
[data.PoseStamped, info.PoseStamped] = ros.internal.ros.messages.geometry_msgs.poseStamped;
info.PoseStamped.MLdataType = 'struct';
[data.CurrentRecoveryNumber, info.CurrentRecoveryNumber] = ros.internal.ros.messages.ros.default_type('uint16',1);
[data.TotalNumberOfRecoveries, info.TotalNumberOfRecoveries] = ros.internal.ros.messages.ros.default_type('uint16',1);
[data.RecoveryBehaviorName, info.RecoveryBehaviorName] = ros.internal.ros.messages.ros.char('string',0);
info.MessageType = 'move_base_msgs/RecoveryStatus';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,20);
info.MatPath{1} = 'pose_stamped';
info.MatPath{2} = 'pose_stamped.header';
info.MatPath{3} = 'pose_stamped.header.seq';
info.MatPath{4} = 'pose_stamped.header.stamp';
info.MatPath{5} = 'pose_stamped.header.stamp.sec';
info.MatPath{6} = 'pose_stamped.header.stamp.nsec';
info.MatPath{7} = 'pose_stamped.header.frame_id';
info.MatPath{8} = 'pose_stamped.pose';
info.MatPath{9} = 'pose_stamped.pose.position';
info.MatPath{10} = 'pose_stamped.pose.position.x';
info.MatPath{11} = 'pose_stamped.pose.position.y';
info.MatPath{12} = 'pose_stamped.pose.position.z';
info.MatPath{13} = 'pose_stamped.pose.orientation';
info.MatPath{14} = 'pose_stamped.pose.orientation.x';
info.MatPath{15} = 'pose_stamped.pose.orientation.y';
info.MatPath{16} = 'pose_stamped.pose.orientation.z';
info.MatPath{17} = 'pose_stamped.pose.orientation.w';
info.MatPath{18} = 'current_recovery_number';
info.MatPath{19} = 'total_number_of_recoveries';
info.MatPath{20} = 'recovery_behavior_name';
