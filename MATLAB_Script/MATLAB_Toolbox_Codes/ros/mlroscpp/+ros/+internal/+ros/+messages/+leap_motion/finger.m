function [data, info] = finger
%Finger gives an empty data for leap_motion/Finger

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'leap_motion/Finger';
[data.Header, info.Header] = ros.internal.ros.messages.std_msgs.header;
info.Header.MLdataType = 'struct';
[data.LmcFingerId, info.LmcFingerId] = ros.internal.ros.messages.ros.default_type('int32',1);
[data.Type, info.Type] = ros.internal.ros.messages.ros.default_type('uint8',1);
[data.Length, info.Length] = ros.internal.ros.messages.ros.default_type('single',1);
[data.Width, info.Width] = ros.internal.ros.messages.ros.default_type('single',1);
[data.ToString, info.ToString] = ros.internal.ros.messages.ros.char('string',0);
[data.BoneList, info.BoneList] = ros.internal.ros.messages.leap_motion.bone;
info.BoneList.MLdataType = 'struct';
info.BoneList.MaxLen = NaN;
info.BoneList.MinLen = 0;
data.BoneList = data.BoneList([],1);
info.MessageType = 'leap_motion/Finger';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,43);
info.MatPath{1} = 'header';
info.MatPath{2} = 'header.seq';
info.MatPath{3} = 'header.stamp';
info.MatPath{4} = 'header.stamp.sec';
info.MatPath{5} = 'header.stamp.nsec';
info.MatPath{6} = 'header.frame_id';
info.MatPath{7} = 'lmc_finger_id';
info.MatPath{8} = 'type';
info.MatPath{9} = 'length';
info.MatPath{10} = 'width';
info.MatPath{11} = 'to_string';
info.MatPath{12} = 'bone_list';
info.MatPath{13} = 'bone_list.header';
info.MatPath{14} = 'bone_list.header.seq';
info.MatPath{15} = 'bone_list.header.stamp';
info.MatPath{16} = 'bone_list.header.stamp.sec';
info.MatPath{17} = 'bone_list.header.stamp.nsec';
info.MatPath{18} = 'bone_list.header.frame_id';
info.MatPath{19} = 'bone_list.type';
info.MatPath{20} = 'bone_list.length';
info.MatPath{21} = 'bone_list.width';
info.MatPath{22} = 'bone_list.to_string';
info.MatPath{23} = 'bone_list.bone_start';
info.MatPath{24} = 'bone_list.bone_start.position';
info.MatPath{25} = 'bone_list.bone_start.position.x';
info.MatPath{26} = 'bone_list.bone_start.position.y';
info.MatPath{27} = 'bone_list.bone_start.position.z';
info.MatPath{28} = 'bone_list.bone_start.orientation';
info.MatPath{29} = 'bone_list.bone_start.orientation.x';
info.MatPath{30} = 'bone_list.bone_start.orientation.y';
info.MatPath{31} = 'bone_list.bone_start.orientation.z';
info.MatPath{32} = 'bone_list.bone_start.orientation.w';
info.MatPath{33} = 'bone_list.bone_end';
info.MatPath{34} = 'bone_list.bone_end.position';
info.MatPath{35} = 'bone_list.bone_end.position.x';
info.MatPath{36} = 'bone_list.bone_end.position.y';
info.MatPath{37} = 'bone_list.bone_end.position.z';
info.MatPath{38} = 'bone_list.bone_end.orientation';
info.MatPath{39} = 'bone_list.bone_end.orientation.x';
info.MatPath{40} = 'bone_list.bone_end.orientation.y';
info.MatPath{41} = 'bone_list.bone_end.orientation.z';
info.MatPath{42} = 'bone_list.bone_end.orientation.w';
info.MatPath{43} = 'bone_list.center';