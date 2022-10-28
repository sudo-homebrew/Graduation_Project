function [data, info] = mmRobotPosition
%MmRobotPosition gives an empty data for adhoc_communication/MmRobotPosition

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'adhoc_communication/MmRobotPosition';
[data.SrcRobot, info.SrcRobot] = ros.internal.ros.messages.ros.char('string',0);
[data.Position, info.Position] = ros.internal.ros.messages.geometry_msgs.poseStamped;
info.Position.MLdataType = 'struct';
info.MessageType = 'adhoc_communication/MmRobotPosition';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,18);
info.MatPath{1} = 'src_robot';
info.MatPath{2} = 'position';
info.MatPath{3} = 'position.header';
info.MatPath{4} = 'position.header.seq';
info.MatPath{5} = 'position.header.stamp';
info.MatPath{6} = 'position.header.stamp.sec';
info.MatPath{7} = 'position.header.stamp.nsec';
info.MatPath{8} = 'position.header.frame_id';
info.MatPath{9} = 'position.pose';
info.MatPath{10} = 'position.pose.position';
info.MatPath{11} = 'position.pose.position.x';
info.MatPath{12} = 'position.pose.position.y';
info.MatPath{13} = 'position.pose.position.z';
info.MatPath{14} = 'position.pose.orientation';
info.MatPath{15} = 'position.pose.orientation.x';
info.MatPath{16} = 'position.pose.orientation.y';
info.MatPath{17} = 'position.pose.orientation.z';
info.MatPath{18} = 'position.pose.orientation.w';
