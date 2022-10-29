function [data, info] = poseWithCovarianceStamped
%PoseWithCovarianceStamped gives an empty data for geometry_msgs/PoseWithCovarianceStamped

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'geometry_msgs/PoseWithCovarianceStamped';
[data.header, info.header] = ros.internal.ros2.messages.std_msgs.header;
info.header.MLdataType = 'struct';
[data.pose, info.pose] = ros.internal.ros2.messages.geometry_msgs.poseWithCovariance;
info.pose.MLdataType = 'struct';
info.MessageType = 'geometry_msgs/PoseWithCovarianceStamped';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,17);
info.MatPath{1} = 'header';
info.MatPath{2} = 'header.stamp';
info.MatPath{3} = 'header.stamp.sec';
info.MatPath{4} = 'header.stamp.nanosec';
info.MatPath{5} = 'header.frame_id';
info.MatPath{6} = 'pose';
info.MatPath{7} = 'pose.pose';
info.MatPath{8} = 'pose.pose.position';
info.MatPath{9} = 'pose.pose.position.x';
info.MatPath{10} = 'pose.pose.position.y';
info.MatPath{11} = 'pose.pose.position.z';
info.MatPath{12} = 'pose.pose.orientation';
info.MatPath{13} = 'pose.pose.orientation.x';
info.MatPath{14} = 'pose.pose.orientation.y';
info.MatPath{15} = 'pose.pose.orientation.z';
info.MatPath{16} = 'pose.pose.orientation.w';
info.MatPath{17} = 'pose.covariance';
