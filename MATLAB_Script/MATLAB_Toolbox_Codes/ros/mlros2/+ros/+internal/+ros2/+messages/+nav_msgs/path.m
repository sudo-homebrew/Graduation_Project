function [data, info] = path
%Path gives an empty data for nav_msgs/Path

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'nav_msgs/Path';
[data.header, info.header] = ros.internal.ros2.messages.std_msgs.header;
info.header.MLdataType = 'struct';
[data.poses, info.poses] = ros.internal.ros2.messages.geometry_msgs.poseStamped;
info.poses.MLdataType = 'struct';
info.poses.MaxLen = NaN;
info.poses.MinLen = 0;
info.MessageType = 'nav_msgs/Path';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,21);
info.MatPath{1} = 'header';
info.MatPath{2} = 'header.stamp';
info.MatPath{3} = 'header.stamp.sec';
info.MatPath{4} = 'header.stamp.nanosec';
info.MatPath{5} = 'header.frame_id';
info.MatPath{6} = 'poses';
info.MatPath{7} = 'poses.header';
info.MatPath{8} = 'poses.header.stamp';
info.MatPath{9} = 'poses.header.stamp.sec';
info.MatPath{10} = 'poses.header.stamp.nanosec';
info.MatPath{11} = 'poses.header.frame_id';
info.MatPath{12} = 'poses.pose';
info.MatPath{13} = 'poses.pose.position';
info.MatPath{14} = 'poses.pose.position.x';
info.MatPath{15} = 'poses.pose.position.y';
info.MatPath{16} = 'poses.pose.position.z';
info.MatPath{17} = 'poses.pose.orientation';
info.MatPath{18} = 'poses.pose.orientation.x';
info.MatPath{19} = 'poses.pose.orientation.y';
info.MatPath{20} = 'poses.pose.orientation.z';
info.MatPath{21} = 'poses.pose.orientation.w';