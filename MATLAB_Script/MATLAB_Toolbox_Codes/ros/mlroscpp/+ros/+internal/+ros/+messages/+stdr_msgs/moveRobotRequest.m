function [data, info] = moveRobotRequest
%MoveRobot gives an empty data for stdr_msgs/MoveRobotRequest

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'stdr_msgs/MoveRobotRequest';
[data.NewPose, info.NewPose] = ros.internal.ros.messages.geometry_msgs.pose2D;
info.NewPose.MLdataType = 'struct';
info.MessageType = 'stdr_msgs/MoveRobotRequest';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,4);
info.MatPath{1} = 'newPose';
info.MatPath{2} = 'newPose.x';
info.MatPath{3} = 'newPose.y';
info.MatPath{4} = 'newPose.theta';
