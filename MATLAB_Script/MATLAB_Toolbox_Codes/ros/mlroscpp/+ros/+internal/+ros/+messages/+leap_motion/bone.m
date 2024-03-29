function [data, info] = bone
%Bone gives an empty data for leap_motion/Bone

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'leap_motion/Bone';
[data.Header, info.Header] = ros.internal.ros.messages.std_msgs.header;
info.Header.MLdataType = 'struct';
[data.Type, info.Type] = ros.internal.ros.messages.ros.default_type('uint8',1);
[data.Length, info.Length] = ros.internal.ros.messages.ros.default_type('single',1);
[data.Width, info.Width] = ros.internal.ros.messages.ros.default_type('single',1);
[data.ToString, info.ToString] = ros.internal.ros.messages.ros.char('string',0);
[data.BoneStart, info.BoneStart] = ros.internal.ros.messages.geometry_msgs.pose;
info.BoneStart.MLdataType = 'struct';
[data.BoneEnd, info.BoneEnd] = ros.internal.ros.messages.geometry_msgs.pose;
info.BoneEnd.MLdataType = 'struct';
[data.Center, info.Center] = ros.internal.ros.messages.ros.default_type('single',NaN);
info.MessageType = 'leap_motion/Bone';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,31);
info.MatPath{1} = 'header';
info.MatPath{2} = 'header.seq';
info.MatPath{3} = 'header.stamp';
info.MatPath{4} = 'header.stamp.sec';
info.MatPath{5} = 'header.stamp.nsec';
info.MatPath{6} = 'header.frame_id';
info.MatPath{7} = 'type';
info.MatPath{8} = 'length';
info.MatPath{9} = 'width';
info.MatPath{10} = 'to_string';
info.MatPath{11} = 'bone_start';
info.MatPath{12} = 'bone_start.position';
info.MatPath{13} = 'bone_start.position.x';
info.MatPath{14} = 'bone_start.position.y';
info.MatPath{15} = 'bone_start.position.z';
info.MatPath{16} = 'bone_start.orientation';
info.MatPath{17} = 'bone_start.orientation.x';
info.MatPath{18} = 'bone_start.orientation.y';
info.MatPath{19} = 'bone_start.orientation.z';
info.MatPath{20} = 'bone_start.orientation.w';
info.MatPath{21} = 'bone_end';
info.MatPath{22} = 'bone_end.position';
info.MatPath{23} = 'bone_end.position.x';
info.MatPath{24} = 'bone_end.position.y';
info.MatPath{25} = 'bone_end.position.z';
info.MatPath{26} = 'bone_end.orientation';
info.MatPath{27} = 'bone_end.orientation.x';
info.MatPath{28} = 'bone_end.orientation.y';
info.MatPath{29} = 'bone_end.orientation.z';
info.MatPath{30} = 'bone_end.orientation.w';
info.MatPath{31} = 'center';
