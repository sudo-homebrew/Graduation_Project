function [data, info] = moveBaseFeedback
%MoveBaseFeedback gives an empty data for move_base_msgs/MoveBaseFeedback

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'move_base_msgs/MoveBaseFeedback';
[data.BasePosition, info.BasePosition] = ros.internal.ros.messages.geometry_msgs.poseStamped;
info.BasePosition.MLdataType = 'struct';
info.MessageType = 'move_base_msgs/MoveBaseFeedback';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,17);
info.MatPath{1} = 'base_position';
info.MatPath{2} = 'base_position.header';
info.MatPath{3} = 'base_position.header.seq';
info.MatPath{4} = 'base_position.header.stamp';
info.MatPath{5} = 'base_position.header.stamp.sec';
info.MatPath{6} = 'base_position.header.stamp.nsec';
info.MatPath{7} = 'base_position.header.frame_id';
info.MatPath{8} = 'base_position.pose';
info.MatPath{9} = 'base_position.pose.position';
info.MatPath{10} = 'base_position.pose.position.x';
info.MatPath{11} = 'base_position.pose.position.y';
info.MatPath{12} = 'base_position.pose.position.z';
info.MatPath{13} = 'base_position.pose.orientation';
info.MatPath{14} = 'base_position.pose.orientation.x';
info.MatPath{15} = 'base_position.pose.orientation.y';
info.MatPath{16} = 'base_position.pose.orientation.z';
info.MatPath{17} = 'base_position.pose.orientation.w';
