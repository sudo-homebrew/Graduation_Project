function [data, info] = poseCommand
%PoseCommand gives an empty data for r2_msgs/PoseCommand

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'r2_msgs/PoseCommand';
[data.CommandId, info.CommandId] = ros.internal.ros.messages.ros.char('string',0);
[data.RefFrame, info.RefFrame] = ros.internal.ros.messages.ros.char('string',0);
[data.Pose, info.Pose] = ros.internal.ros.messages.geometry_msgs.pose;
info.Pose.MLdataType = 'struct';
[data.BaseFrame, info.BaseFrame] = ros.internal.ros.messages.ros.char('string',0);
[data.ToolFrame, info.ToolFrame] = ros.internal.ros.messages.ros.char('string',0);
info.MessageType = 'r2_msgs/PoseCommand';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,14);
info.MatPath{1} = 'commandId';
info.MatPath{2} = 'refFrame';
info.MatPath{3} = 'pose';
info.MatPath{4} = 'pose.position';
info.MatPath{5} = 'pose.position.x';
info.MatPath{6} = 'pose.position.y';
info.MatPath{7} = 'pose.position.z';
info.MatPath{8} = 'pose.orientation';
info.MatPath{9} = 'pose.orientation.x';
info.MatPath{10} = 'pose.orientation.y';
info.MatPath{11} = 'pose.orientation.z';
info.MatPath{12} = 'pose.orientation.w';
info.MatPath{13} = 'baseFrame';
info.MatPath{14} = 'toolFrame';
