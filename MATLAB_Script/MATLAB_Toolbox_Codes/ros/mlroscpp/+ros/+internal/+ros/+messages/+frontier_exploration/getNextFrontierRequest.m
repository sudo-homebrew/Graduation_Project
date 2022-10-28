function [data, info] = getNextFrontierRequest
%GetNextFrontier gives an empty data for frontier_exploration/GetNextFrontierRequest

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'frontier_exploration/GetNextFrontierRequest';
[data.StartPose, info.StartPose] = ros.internal.ros.messages.geometry_msgs.poseStamped;
info.StartPose.MLdataType = 'struct';
info.MessageType = 'frontier_exploration/GetNextFrontierRequest';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,17);
info.MatPath{1} = 'start_pose';
info.MatPath{2} = 'start_pose.header';
info.MatPath{3} = 'start_pose.header.seq';
info.MatPath{4} = 'start_pose.header.stamp';
info.MatPath{5} = 'start_pose.header.stamp.sec';
info.MatPath{6} = 'start_pose.header.stamp.nsec';
info.MatPath{7} = 'start_pose.header.frame_id';
info.MatPath{8} = 'start_pose.pose';
info.MatPath{9} = 'start_pose.pose.position';
info.MatPath{10} = 'start_pose.pose.position.x';
info.MatPath{11} = 'start_pose.pose.position.y';
info.MatPath{12} = 'start_pose.pose.position.z';
info.MatPath{13} = 'start_pose.pose.orientation';
info.MatPath{14} = 'start_pose.pose.orientation.x';
info.MatPath{15} = 'start_pose.pose.orientation.y';
info.MatPath{16} = 'start_pose.pose.orientation.z';
info.MatPath{17} = 'start_pose.pose.orientation.w';
