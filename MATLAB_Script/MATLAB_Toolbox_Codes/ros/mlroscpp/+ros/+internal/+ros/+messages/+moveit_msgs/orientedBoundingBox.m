function [data, info] = orientedBoundingBox
%OrientedBoundingBox gives an empty data for moveit_msgs/OrientedBoundingBox

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'moveit_msgs/OrientedBoundingBox';
[data.Pose, info.Pose] = ros.internal.ros.messages.geometry_msgs.pose;
info.Pose.MLdataType = 'struct';
[data.Extents, info.Extents] = ros.internal.ros.messages.geometry_msgs.point32;
info.Extents.MLdataType = 'struct';
info.MessageType = 'moveit_msgs/OrientedBoundingBox';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,14);
info.MatPath{1} = 'pose';
info.MatPath{2} = 'pose.position';
info.MatPath{3} = 'pose.position.x';
info.MatPath{4} = 'pose.position.y';
info.MatPath{5} = 'pose.position.z';
info.MatPath{6} = 'pose.orientation';
info.MatPath{7} = 'pose.orientation.x';
info.MatPath{8} = 'pose.orientation.y';
info.MatPath{9} = 'pose.orientation.z';
info.MatPath{10} = 'pose.orientation.w';
info.MatPath{11} = 'extents';
info.MatPath{12} = 'extents.x';
info.MatPath{13} = 'extents.y';
info.MatPath{14} = 'extents.z';
