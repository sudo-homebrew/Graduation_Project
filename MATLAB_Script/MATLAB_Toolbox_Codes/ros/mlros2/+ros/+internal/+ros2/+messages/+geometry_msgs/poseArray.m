function [data, info] = poseArray
%PoseArray gives an empty data for geometry_msgs/PoseArray

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'geometry_msgs/PoseArray';
[data.header, info.header] = ros.internal.ros2.messages.std_msgs.header;
info.header.MLdataType = 'struct';
[data.poses, info.poses] = ros.internal.ros2.messages.geometry_msgs.pose;
info.poses.MLdataType = 'struct';
info.poses.MaxLen = NaN;
info.poses.MinLen = 0;
info.MessageType = 'geometry_msgs/PoseArray';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,15);
info.MatPath{1} = 'header';
info.MatPath{2} = 'header.stamp';
info.MatPath{3} = 'header.stamp.sec';
info.MatPath{4} = 'header.stamp.nanosec';
info.MatPath{5} = 'header.frame_id';
info.MatPath{6} = 'poses';
info.MatPath{7} = 'poses.position';
info.MatPath{8} = 'poses.position.x';
info.MatPath{9} = 'poses.position.y';
info.MatPath{10} = 'poses.position.z';
info.MatPath{11} = 'poses.orientation';
info.MatPath{12} = 'poses.orientation.x';
info.MatPath{13} = 'poses.orientation.y';
info.MatPath{14} = 'poses.orientation.z';
info.MatPath{15} = 'poses.orientation.w';
