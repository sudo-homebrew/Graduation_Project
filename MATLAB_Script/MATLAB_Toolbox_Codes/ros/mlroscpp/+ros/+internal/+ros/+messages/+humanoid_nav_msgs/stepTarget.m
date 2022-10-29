function [data, info] = stepTarget
%StepTarget gives an empty data for humanoid_nav_msgs/StepTarget

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'humanoid_nav_msgs/StepTarget';
[data.Pose, info.Pose] = ros.internal.ros.messages.geometry_msgs.pose2D;
info.Pose.MLdataType = 'struct';
[data.Leg, info.Leg] = ros.internal.ros.messages.ros.default_type('uint8',1);
[data.Right, info.Right] = ros.internal.ros.messages.ros.default_type('uint8',1, 0);
[data.Left, info.Left] = ros.internal.ros.messages.ros.default_type('uint8',1, 1);
info.MessageType = 'humanoid_nav_msgs/StepTarget';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,7);
info.MatPath{1} = 'pose';
info.MatPath{2} = 'pose.x';
info.MatPath{3} = 'pose.y';
info.MatPath{4} = 'pose.theta';
info.MatPath{5} = 'leg';
info.MatPath{6} = 'right';
info.MatPath{7} = 'left';
