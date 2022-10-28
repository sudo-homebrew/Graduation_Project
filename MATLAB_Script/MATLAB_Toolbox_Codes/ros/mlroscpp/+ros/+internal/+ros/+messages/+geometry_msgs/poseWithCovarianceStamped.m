function [data, info] = poseWithCovarianceStamped
%PoseWithCovarianceStamped gives an empty data for geometry_msgs/PoseWithCovarianceStamped

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'geometry_msgs/PoseWithCovarianceStamped';
[data.Header, info.Header] = ros.internal.ros.messages.std_msgs.header;
info.Header.MLdataType = 'struct';
[data.Pose, info.Pose] = ros.internal.ros.messages.geometry_msgs.poseWithCovariance;
info.Pose.MLdataType = 'struct';
info.MessageType = 'geometry_msgs/PoseWithCovarianceStamped';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,18);
info.MatPath{1} = 'header';
info.MatPath{2} = 'header.seq';
info.MatPath{3} = 'header.stamp';
info.MatPath{4} = 'header.stamp.sec';
info.MatPath{5} = 'header.stamp.nsec';
info.MatPath{6} = 'header.frame_id';
info.MatPath{7} = 'pose';
info.MatPath{8} = 'pose.pose';
info.MatPath{9} = 'pose.pose.position';
info.MatPath{10} = 'pose.pose.position.x';
info.MatPath{11} = 'pose.pose.position.y';
info.MatPath{12} = 'pose.pose.position.z';
info.MatPath{13} = 'pose.pose.orientation';
info.MatPath{14} = 'pose.pose.orientation.x';
info.MatPath{15} = 'pose.pose.orientation.y';
info.MatPath{16} = 'pose.pose.orientation.z';
info.MatPath{17} = 'pose.pose.orientation.w';
info.MatPath{18} = 'pose.covariance';
