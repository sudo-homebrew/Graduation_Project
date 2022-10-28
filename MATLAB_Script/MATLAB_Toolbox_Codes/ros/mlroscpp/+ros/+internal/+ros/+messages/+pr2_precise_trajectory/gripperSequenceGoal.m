function [data, info] = gripperSequenceGoal
%GripperSequenceGoal gives an empty data for pr2_precise_trajectory/GripperSequenceGoal

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'pr2_precise_trajectory/GripperSequenceGoal';
[data.Header, info.Header] = ros.internal.ros.messages.std_msgs.header;
info.Header.MLdataType = 'struct';
[data.Positions, info.Positions] = ros.internal.ros.messages.ros.default_type('single',NaN);
[data.Times, info.Times] = ros.internal.ros.messages.ros.default_type('single',NaN);
info.MessageType = 'pr2_precise_trajectory/GripperSequenceGoal';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,8);
info.MatPath{1} = 'header';
info.MatPath{2} = 'header.seq';
info.MatPath{3} = 'header.stamp';
info.MatPath{4} = 'header.stamp.sec';
info.MatPath{5} = 'header.stamp.nsec';
info.MatPath{6} = 'header.frame_id';
info.MatPath{7} = 'positions';
info.MatPath{8} = 'times';
