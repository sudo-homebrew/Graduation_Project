function [data, info] = table
%Table gives an empty data for object_recognition_msgs/Table

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'object_recognition_msgs/Table';
[data.Header, info.Header] = ros.internal.ros.messages.std_msgs.header;
info.Header.MLdataType = 'struct';
[data.Pose, info.Pose] = ros.internal.ros.messages.geometry_msgs.pose;
info.Pose.MLdataType = 'struct';
[data.ConvexHull, info.ConvexHull] = ros.internal.ros.messages.geometry_msgs.point;
info.ConvexHull.MLdataType = 'struct';
info.ConvexHull.MaxLen = NaN;
info.ConvexHull.MinLen = 0;
data.ConvexHull = data.ConvexHull([],1);
info.MessageType = 'object_recognition_msgs/Table';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,20);
info.MatPath{1} = 'header';
info.MatPath{2} = 'header.seq';
info.MatPath{3} = 'header.stamp';
info.MatPath{4} = 'header.stamp.sec';
info.MatPath{5} = 'header.stamp.nsec';
info.MatPath{6} = 'header.frame_id';
info.MatPath{7} = 'pose';
info.MatPath{8} = 'pose.position';
info.MatPath{9} = 'pose.position.x';
info.MatPath{10} = 'pose.position.y';
info.MatPath{11} = 'pose.position.z';
info.MatPath{12} = 'pose.orientation';
info.MatPath{13} = 'pose.orientation.x';
info.MatPath{14} = 'pose.orientation.y';
info.MatPath{15} = 'pose.orientation.z';
info.MatPath{16} = 'pose.orientation.w';
info.MatPath{17} = 'convex_hull';
info.MatPath{18} = 'convex_hull.x';
info.MatPath{19} = 'convex_hull.y';
info.MatPath{20} = 'convex_hull.z';