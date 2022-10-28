function [data, info] = geoPoseStamped
%GeoPoseStamped gives an empty data for geographic_msgs/GeoPoseStamped

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'geographic_msgs/GeoPoseStamped';
[data.Header, info.Header] = ros.internal.ros.messages.std_msgs.header;
info.Header.MLdataType = 'struct';
[data.Pose, info.Pose] = ros.internal.ros.messages.geographic_msgs.geoPose;
info.Pose.MLdataType = 'struct';
info.MessageType = 'geographic_msgs/GeoPoseStamped';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,16);
info.MatPath{1} = 'header';
info.MatPath{2} = 'header.seq';
info.MatPath{3} = 'header.stamp';
info.MatPath{4} = 'header.stamp.sec';
info.MatPath{5} = 'header.stamp.nsec';
info.MatPath{6} = 'header.frame_id';
info.MatPath{7} = 'pose';
info.MatPath{8} = 'pose.position';
info.MatPath{9} = 'pose.position.latitude';
info.MatPath{10} = 'pose.position.longitude';
info.MatPath{11} = 'pose.position.altitude';
info.MatPath{12} = 'pose.orientation';
info.MatPath{13} = 'pose.orientation.x';
info.MatPath{14} = 'pose.orientation.y';
info.MatPath{15} = 'pose.orientation.z';
info.MatPath{16} = 'pose.orientation.w';
