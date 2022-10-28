function [data, info] = targetObjResponse
%TargetObj gives an empty data for posedetection_msgs/TargetObjResponse

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'posedetection_msgs/TargetObjResponse';
[data.ObjectPose, info.ObjectPose] = ros.internal.ros.messages.posedetection_msgs.object6DPose;
info.ObjectPose.MLdataType = 'struct';
info.MessageType = 'posedetection_msgs/TargetObjResponse';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,13);
info.MatPath{1} = 'object_pose';
info.MatPath{2} = 'object_pose.pose';
info.MatPath{3} = 'object_pose.pose.position';
info.MatPath{4} = 'object_pose.pose.position.x';
info.MatPath{5} = 'object_pose.pose.position.y';
info.MatPath{6} = 'object_pose.pose.position.z';
info.MatPath{7} = 'object_pose.pose.orientation';
info.MatPath{8} = 'object_pose.pose.orientation.x';
info.MatPath{9} = 'object_pose.pose.orientation.y';
info.MatPath{10} = 'object_pose.pose.orientation.z';
info.MatPath{11} = 'object_pose.pose.orientation.w';
info.MatPath{12} = 'object_pose.reliability';
info.MatPath{13} = 'object_pose.type';
