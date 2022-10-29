function [data, info] = interactiveMarkerPose
%InteractiveMarkerPose gives an empty data for visualization_msgs/InteractiveMarkerPose

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'visualization_msgs/InteractiveMarkerPose';
[data.Header, info.Header] = ros.internal.ros.messages.std_msgs.header;
info.Header.MLdataType = 'struct';
[data.Pose, info.Pose] = ros.internal.ros.messages.geometry_msgs.pose;
info.Pose.MLdataType = 'struct';
[data.Name, info.Name] = ros.internal.ros.messages.ros.char('string',0);
info.MessageType = 'visualization_msgs/InteractiveMarkerPose';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,17);
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
info.MatPath{17} = 'name';
