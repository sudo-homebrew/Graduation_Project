function [data, info] = moveSequenceGoal
%MoveSequenceGoal gives an empty data for pr2_precise_trajectory/MoveSequenceGoal

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'pr2_precise_trajectory/MoveSequenceGoal';
[data.Header, info.Header] = ros.internal.ros.messages.std_msgs.header;
info.Header.MLdataType = 'struct';
[data.Poses, info.Poses] = ros.internal.ros.messages.geometry_msgs.pose;
info.Poses.MLdataType = 'struct';
info.Poses.MaxLen = NaN;
info.Poses.MinLen = 0;
data.Poses = data.Poses([],1);
[data.Times, info.Times] = ros.internal.ros.messages.ros.default_type('single',NaN);
info.MessageType = 'pr2_precise_trajectory/MoveSequenceGoal';
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
info.MatPath{7} = 'poses';
info.MatPath{8} = 'poses.position';
info.MatPath{9} = 'poses.position.x';
info.MatPath{10} = 'poses.position.y';
info.MatPath{11} = 'poses.position.z';
info.MatPath{12} = 'poses.orientation';
info.MatPath{13} = 'poses.orientation.x';
info.MatPath{14} = 'poses.orientation.y';
info.MatPath{15} = 'poses.orientation.z';
info.MatPath{16} = 'poses.orientation.w';
info.MatPath{17} = 'times';
