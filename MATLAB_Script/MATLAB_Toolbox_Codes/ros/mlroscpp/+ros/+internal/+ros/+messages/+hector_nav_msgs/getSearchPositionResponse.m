function [data, info] = getSearchPositionResponse
%GetSearchPosition gives an empty data for hector_nav_msgs/GetSearchPositionResponse

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'hector_nav_msgs/GetSearchPositionResponse';
[data.SearchPose, info.SearchPose] = ros.internal.ros.messages.geometry_msgs.poseStamped;
info.SearchPose.MLdataType = 'struct';
info.MessageType = 'hector_nav_msgs/GetSearchPositionResponse';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,17);
info.MatPath{1} = 'search_pose';
info.MatPath{2} = 'search_pose.header';
info.MatPath{3} = 'search_pose.header.seq';
info.MatPath{4} = 'search_pose.header.stamp';
info.MatPath{5} = 'search_pose.header.stamp.sec';
info.MatPath{6} = 'search_pose.header.stamp.nsec';
info.MatPath{7} = 'search_pose.header.frame_id';
info.MatPath{8} = 'search_pose.pose';
info.MatPath{9} = 'search_pose.pose.position';
info.MatPath{10} = 'search_pose.pose.position.x';
info.MatPath{11} = 'search_pose.pose.position.y';
info.MatPath{12} = 'search_pose.pose.position.z';
info.MatPath{13} = 'search_pose.pose.orientation';
info.MatPath{14} = 'search_pose.pose.orientation.x';
info.MatPath{15} = 'search_pose.pose.orientation.y';
info.MatPath{16} = 'search_pose.pose.orientation.z';
info.MatPath{17} = 'search_pose.pose.orientation.w';
