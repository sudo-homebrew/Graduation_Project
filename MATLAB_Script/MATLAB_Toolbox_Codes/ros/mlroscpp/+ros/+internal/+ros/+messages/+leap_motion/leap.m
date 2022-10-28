function [data, info] = leap
%leap gives an empty data for leap_motion/leap

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'leap_motion/leap';
[data.Header, info.Header] = ros.internal.ros.messages.std_msgs.header;
info.Header.MLdataType = 'struct';
[data.HandDirection, info.HandDirection] = ros.internal.ros.messages.ros.default_type('double',3);
[data.HandNormal, info.HandNormal] = ros.internal.ros.messages.ros.default_type('double',3);
[data.HandPalmPos, info.HandPalmPos] = ros.internal.ros.messages.ros.default_type('double',3);
[data.HandPitch, info.HandPitch] = ros.internal.ros.messages.ros.default_type('double',1);
[data.HandRoll, info.HandRoll] = ros.internal.ros.messages.ros.default_type('double',1);
[data.HandYaw, info.HandYaw] = ros.internal.ros.messages.ros.default_type('double',1);
info.MessageType = 'leap_motion/leap';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,12);
info.MatPath{1} = 'header';
info.MatPath{2} = 'header.seq';
info.MatPath{3} = 'header.stamp';
info.MatPath{4} = 'header.stamp.sec';
info.MatPath{5} = 'header.stamp.nsec';
info.MatPath{6} = 'header.frame_id';
info.MatPath{7} = 'hand_direction';
info.MatPath{8} = 'hand_normal';
info.MatPath{9} = 'hand_palm_pos';
info.MatPath{10} = 'hand_pitch';
info.MatPath{11} = 'hand_roll';
info.MatPath{12} = 'hand_yaw';
