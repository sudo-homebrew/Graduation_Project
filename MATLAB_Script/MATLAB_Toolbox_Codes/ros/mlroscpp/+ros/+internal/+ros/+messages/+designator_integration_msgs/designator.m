function [data, info] = designator
%Designator gives an empty data for designator_integration_msgs/Designator

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'designator_integration_msgs/Designator';
[data.TYPEOBJECT, info.TYPEOBJECT] = ros.internal.ros.messages.ros.default_type('int32',1, 0);
[data.TYPEACTION, info.TYPEACTION] = ros.internal.ros.messages.ros.default_type('int32',1, 1);
[data.TYPELOCATION, info.TYPELOCATION] = ros.internal.ros.messages.ros.default_type('int32',1, 2);
[data.TYPEHUMAN, info.TYPEHUMAN] = ros.internal.ros.messages.ros.default_type('int32',1, 3);
[data.Type, info.Type] = ros.internal.ros.messages.ros.default_type('int32',1);
[data.Description, info.Description] = ros.internal.ros.messages.designator_integration_msgs.keyValuePair;
info.Description.MLdataType = 'struct';
info.Description.MaxLen = NaN;
info.Description.MinLen = 0;
data.Description = data.Description([],1);
info.MessageType = 'designator_integration_msgs/Designator';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,68);
info.MatPath{1} = 'TYPE_OBJECT';
info.MatPath{2} = 'TYPE_ACTION';
info.MatPath{3} = 'TYPE_LOCATION';
info.MatPath{4} = 'TYPE_HUMAN';
info.MatPath{5} = 'type';
info.MatPath{6} = 'description';
info.MatPath{7} = 'description.id';
info.MatPath{8} = 'description.parent';
info.MatPath{9} = 'description.TYPE_STRING';
info.MatPath{10} = 'description.TYPE_FLOAT';
info.MatPath{11} = 'description.TYPE_DATA';
info.MatPath{12} = 'description.TYPE_LIST';
info.MatPath{13} = 'description.TYPE_POSESTAMPED';
info.MatPath{14} = 'description.TYPE_POSE';
info.MatPath{15} = 'description.TYPE_DESIGNATOR_ACTION';
info.MatPath{16} = 'description.TYPE_DESIGNATOR_OBJECT';
info.MatPath{17} = 'description.TYPE_DESIGNATOR_LOCATION';
info.MatPath{18} = 'description.TYPE_DESIGNATOR_HUMAN';
info.MatPath{19} = 'description.TYPE_POINT';
info.MatPath{20} = 'description.TYPE_WRENCH';
info.MatPath{21} = 'description.TYPE_MATRIX';
info.MatPath{22} = 'description.TYPE_VECTOR';
info.MatPath{23} = 'description.type';
info.MatPath{24} = 'description.key';
info.MatPath{25} = 'description.value_string';
info.MatPath{26} = 'description.value_float';
info.MatPath{27} = 'description.value_data';
info.MatPath{28} = 'description.value_array';
info.MatPath{29} = 'description.value_posestamped';
info.MatPath{30} = 'description.value_posestamped.header';
info.MatPath{31} = 'description.value_posestamped.header.seq';
info.MatPath{32} = 'description.value_posestamped.header.stamp';
info.MatPath{33} = 'description.value_posestamped.header.stamp.sec';
info.MatPath{34} = 'description.value_posestamped.header.stamp.nsec';
info.MatPath{35} = 'description.value_posestamped.header.frame_id';
info.MatPath{36} = 'description.value_posestamped.pose';
info.MatPath{37} = 'description.value_posestamped.pose.position';
info.MatPath{38} = 'description.value_posestamped.pose.position.x';
info.MatPath{39} = 'description.value_posestamped.pose.position.y';
info.MatPath{40} = 'description.value_posestamped.pose.position.z';
info.MatPath{41} = 'description.value_posestamped.pose.orientation';
info.MatPath{42} = 'description.value_posestamped.pose.orientation.x';
info.MatPath{43} = 'description.value_posestamped.pose.orientation.y';
info.MatPath{44} = 'description.value_posestamped.pose.orientation.z';
info.MatPath{45} = 'description.value_posestamped.pose.orientation.w';
info.MatPath{46} = 'description.value_pose';
info.MatPath{47} = 'description.value_pose.position';
info.MatPath{48} = 'description.value_pose.position.x';
info.MatPath{49} = 'description.value_pose.position.y';
info.MatPath{50} = 'description.value_pose.position.z';
info.MatPath{51} = 'description.value_pose.orientation';
info.MatPath{52} = 'description.value_pose.orientation.x';
info.MatPath{53} = 'description.value_pose.orientation.y';
info.MatPath{54} = 'description.value_pose.orientation.z';
info.MatPath{55} = 'description.value_pose.orientation.w';
info.MatPath{56} = 'description.value_point';
info.MatPath{57} = 'description.value_point.x';
info.MatPath{58} = 'description.value_point.y';
info.MatPath{59} = 'description.value_point.z';
info.MatPath{60} = 'description.value_wrench';
info.MatPath{61} = 'description.value_wrench.force';
info.MatPath{62} = 'description.value_wrench.force.x';
info.MatPath{63} = 'description.value_wrench.force.y';
info.MatPath{64} = 'description.value_wrench.force.z';
info.MatPath{65} = 'description.value_wrench.torque';
info.MatPath{66} = 'description.value_wrench.torque.x';
info.MatPath{67} = 'description.value_wrench.torque.y';
info.MatPath{68} = 'description.value_wrench.torque.z';