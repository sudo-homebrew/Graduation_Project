function [data, info] = getSearchPositionRequest
%GetSearchPosition gives an empty data for hector_nav_msgs/GetSearchPositionRequest

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'hector_nav_msgs/GetSearchPositionRequest';
[data.OoiPose, info.OoiPose] = ros.internal.ros.messages.geometry_msgs.poseStamped;
info.OoiPose.MLdataType = 'struct';
[data.Distance, info.Distance] = ros.internal.ros.messages.ros.default_type('single',1);
info.MessageType = 'hector_nav_msgs/GetSearchPositionRequest';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,18);
info.MatPath{1} = 'ooi_pose';
info.MatPath{2} = 'ooi_pose.header';
info.MatPath{3} = 'ooi_pose.header.seq';
info.MatPath{4} = 'ooi_pose.header.stamp';
info.MatPath{5} = 'ooi_pose.header.stamp.sec';
info.MatPath{6} = 'ooi_pose.header.stamp.nsec';
info.MatPath{7} = 'ooi_pose.header.frame_id';
info.MatPath{8} = 'ooi_pose.pose';
info.MatPath{9} = 'ooi_pose.pose.position';
info.MatPath{10} = 'ooi_pose.pose.position.x';
info.MatPath{11} = 'ooi_pose.pose.position.y';
info.MatPath{12} = 'ooi_pose.pose.position.z';
info.MatPath{13} = 'ooi_pose.pose.orientation';
info.MatPath{14} = 'ooi_pose.pose.orientation.x';
info.MatPath{15} = 'ooi_pose.pose.orientation.y';
info.MatPath{16} = 'ooi_pose.pose.orientation.z';
info.MatPath{17} = 'ooi_pose.pose.orientation.w';
info.MatPath{18} = 'distance';
