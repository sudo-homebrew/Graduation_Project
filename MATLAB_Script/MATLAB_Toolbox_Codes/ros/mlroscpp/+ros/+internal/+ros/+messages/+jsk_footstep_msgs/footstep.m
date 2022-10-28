function [data, info] = footstep
%Footstep gives an empty data for jsk_footstep_msgs/Footstep

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'jsk_footstep_msgs/Footstep';
[data.RIGHT, info.RIGHT] = ros.internal.ros.messages.ros.default_type('uint8',1, 2);
[data.LEFT, info.LEFT] = ros.internal.ros.messages.ros.default_type('uint8',1, 1);
[data.Leg, info.Leg] = ros.internal.ros.messages.ros.default_type('uint8',1);
[data.Pose, info.Pose] = ros.internal.ros.messages.geometry_msgs.pose;
info.Pose.MLdataType = 'struct';
[data.Duration, info.Duration] = ros.internal.ros.messages.ros.duration;
info.Duration.MLdataType = 'struct';
info.MessageType = 'jsk_footstep_msgs/Footstep';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,16);
info.MatPath{1} = 'RIGHT';
info.MatPath{2} = 'LEFT';
info.MatPath{3} = 'leg';
info.MatPath{4} = 'pose';
info.MatPath{5} = 'pose.position';
info.MatPath{6} = 'pose.position.x';
info.MatPath{7} = 'pose.position.y';
info.MatPath{8} = 'pose.position.z';
info.MatPath{9} = 'pose.orientation';
info.MatPath{10} = 'pose.orientation.x';
info.MatPath{11} = 'pose.orientation.y';
info.MatPath{12} = 'pose.orientation.z';
info.MatPath{13} = 'pose.orientation.w';
info.MatPath{14} = 'duration';
info.MatPath{15} = 'duration.sec';
info.MatPath{16} = 'duration.nsec';
