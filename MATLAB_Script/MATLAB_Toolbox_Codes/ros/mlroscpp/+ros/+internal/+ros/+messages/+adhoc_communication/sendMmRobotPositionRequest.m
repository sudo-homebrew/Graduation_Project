function [data, info] = sendMmRobotPositionRequest
%SendMmRobotPosition gives an empty data for adhoc_communication/SendMmRobotPositionRequest

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'adhoc_communication/SendMmRobotPositionRequest';
[data.DstRobot, info.DstRobot] = ros.internal.ros.messages.ros.char('string',0);
[data.Topic, info.Topic] = ros.internal.ros.messages.ros.char('string',0);
[data.Position, info.Position] = ros.internal.ros.messages.adhoc_communication.mmRobotPosition;
info.Position.MLdataType = 'struct';
info.MessageType = 'adhoc_communication/SendMmRobotPositionRequest';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,21);
info.MatPath{1} = 'dst_robot';
info.MatPath{2} = 'topic';
info.MatPath{3} = 'position';
info.MatPath{4} = 'position.src_robot';
info.MatPath{5} = 'position.position';
info.MatPath{6} = 'position.position.header';
info.MatPath{7} = 'position.position.header.seq';
info.MatPath{8} = 'position.position.header.stamp';
info.MatPath{9} = 'position.position.header.stamp.sec';
info.MatPath{10} = 'position.position.header.stamp.nsec';
info.MatPath{11} = 'position.position.header.frame_id';
info.MatPath{12} = 'position.position.pose';
info.MatPath{13} = 'position.position.pose.position';
info.MatPath{14} = 'position.position.pose.position.x';
info.MatPath{15} = 'position.position.pose.position.y';
info.MatPath{16} = 'position.position.pose.position.z';
info.MatPath{17} = 'position.position.pose.orientation';
info.MatPath{18} = 'position.position.pose.orientation.x';
info.MatPath{19} = 'position.position.pose.orientation.y';
info.MatPath{20} = 'position.position.pose.orientation.z';
info.MatPath{21} = 'position.position.pose.orientation.w';
