function [data, info] = interactiveMarkerPose
%InteractiveMarkerPose gives an empty data for visualization_msgs/InteractiveMarkerPose

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'visualization_msgs/InteractiveMarkerPose';
[data.header, info.header] = ros.internal.ros2.messages.std_msgs.header;
info.header.MLdataType = 'struct';
[data.pose, info.pose] = ros.internal.ros2.messages.geometry_msgs.pose;
info.pose.MLdataType = 'struct';
[data.name, info.name] = ros.internal.ros2.messages.ros2.char('string',1,NaN,0);
info.MessageType = 'visualization_msgs/InteractiveMarkerPose';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,16);
info.MatPath{1} = 'header';
info.MatPath{2} = 'header.stamp';
info.MatPath{3} = 'header.stamp.sec';
info.MatPath{4} = 'header.stamp.nanosec';
info.MatPath{5} = 'header.frame_id';
info.MatPath{6} = 'pose';
info.MatPath{7} = 'pose.position';
info.MatPath{8} = 'pose.position.x';
info.MatPath{9} = 'pose.position.y';
info.MatPath{10} = 'pose.position.z';
info.MatPath{11} = 'pose.orientation';
info.MatPath{12} = 'pose.orientation.x';
info.MatPath{13} = 'pose.orientation.y';
info.MatPath{14} = 'pose.orientation.z';
info.MatPath{15} = 'pose.orientation.w';
info.MatPath{16} = 'name';
