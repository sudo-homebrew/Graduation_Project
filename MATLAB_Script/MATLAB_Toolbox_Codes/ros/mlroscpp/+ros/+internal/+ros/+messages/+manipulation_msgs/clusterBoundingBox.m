function [data, info] = clusterBoundingBox
%ClusterBoundingBox gives an empty data for manipulation_msgs/ClusterBoundingBox

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'manipulation_msgs/ClusterBoundingBox';
[data.PoseStamped, info.PoseStamped] = ros.internal.ros.messages.geometry_msgs.poseStamped;
info.PoseStamped.MLdataType = 'struct';
[data.Dimensions, info.Dimensions] = ros.internal.ros.messages.geometry_msgs.vector3;
info.Dimensions.MLdataType = 'struct';
info.MessageType = 'manipulation_msgs/ClusterBoundingBox';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,21);
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
info.MatPath{18} = 'dimensions';
info.MatPath{19} = 'dimensions.x';
info.MatPath{20} = 'dimensions.y';
info.MatPath{21} = 'dimensions.z';
