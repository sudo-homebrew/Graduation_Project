function [data, info] = robotPose
%RobotPose gives an empty data for nav2d_msgs/RobotPose

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'nav2d_msgs/RobotPose';
[data.Header, info.Header] = ros.internal.ros.messages.std_msgs.header;
info.Header.MLdataType = 'struct';
[data.RobotId, info.RobotId] = ros.internal.ros.messages.ros.default_type('int32',1);
[data.Pose, info.Pose] = ros.internal.ros.messages.geometry_msgs.pose2D;
info.Pose.MLdataType = 'struct';
info.MessageType = 'nav2d_msgs/RobotPose';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,11);
info.MatPath{1} = 'header';
info.MatPath{2} = 'header.seq';
info.MatPath{3} = 'header.stamp';
info.MatPath{4} = 'header.stamp.sec';
info.MatPath{5} = 'header.stamp.nsec';
info.MatPath{6} = 'header.frame_id';
info.MatPath{7} = 'robot_id';
info.MatPath{8} = 'pose';
info.MatPath{9} = 'pose.x';
info.MatPath{10} = 'pose.y';
info.MatPath{11} = 'pose.theta';
