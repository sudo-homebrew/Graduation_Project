function [data, info] = getPositionFKRequest
%GetPositionFK gives an empty data for moveit_msgs/GetPositionFKRequest

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'moveit_msgs/GetPositionFKRequest';
[data.Header, info.Header] = ros.internal.ros.messages.std_msgs.header;
info.Header.MLdataType = 'struct';
[data.FkLinkNames, info.FkLinkNames] = ros.internal.ros.messages.ros.char('string',NaN);
[data.RobotState, info.RobotState] = ros.internal.ros.messages.moveit_msgs.robotState;
info.RobotState.MLdataType = 'struct';
info.MessageType = 'moveit_msgs/GetPositionFKRequest';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,167);
info.MatPath{1} = 'header';
info.MatPath{2} = 'header.seq';
info.MatPath{3} = 'header.stamp';
info.MatPath{4} = 'header.stamp.sec';
info.MatPath{5} = 'header.stamp.nsec';
info.MatPath{6} = 'header.frame_id';
info.MatPath{7} = 'fk_link_names';
info.MatPath{8} = 'robot_state';
info.MatPath{9} = 'robot_state.joint_state';
info.MatPath{10} = 'robot_state.joint_state.header';
info.MatPath{11} = 'robot_state.joint_state.header.seq';
info.MatPath{12} = 'robot_state.joint_state.header.stamp';
info.MatPath{13} = 'robot_state.joint_state.header.stamp.sec';
info.MatPath{14} = 'robot_state.joint_state.header.stamp.nsec';
info.MatPath{15} = 'robot_state.joint_state.header.frame_id';
info.MatPath{16} = 'robot_state.joint_state.name';
info.MatPath{17} = 'robot_state.joint_state.position';
info.MatPath{18} = 'robot_state.joint_state.velocity';
info.MatPath{19} = 'robot_state.joint_state.effort';
info.MatPath{20} = 'robot_state.multi_dof_joint_state';
info.MatPath{21} = 'robot_state.multi_dof_joint_state.header';
info.MatPath{22} = 'robot_state.multi_dof_joint_state.header.seq';
info.MatPath{23} = 'robot_state.multi_dof_joint_state.header.stamp';
info.MatPath{24} = 'robot_state.multi_dof_joint_state.header.stamp.sec';
info.MatPath{25} = 'robot_state.multi_dof_joint_state.header.stamp.nsec';
info.MatPath{26} = 'robot_state.multi_dof_joint_state.header.frame_id';
info.MatPath{27} = 'robot_state.multi_dof_joint_state.joint_names';
info.MatPath{28} = 'robot_state.multi_dof_joint_state.transforms';
info.MatPath{29} = 'robot_state.multi_dof_joint_state.transforms.translation';
info.MatPath{30} = 'robot_state.multi_dof_joint_state.transforms.translation.x';
info.MatPath{31} = 'robot_state.multi_dof_joint_state.transforms.translation.y';
info.MatPath{32} = 'robot_state.multi_dof_joint_state.transforms.translation.z';
info.MatPath{33} = 'robot_state.multi_dof_joint_state.transforms.rotation';
info.MatPath{34} = 'robot_state.multi_dof_joint_state.transforms.rotation.x';
info.MatPath{35} = 'robot_state.multi_dof_joint_state.transforms.rotation.y';
info.MatPath{36} = 'robot_state.multi_dof_joint_state.transforms.rotation.z';
info.MatPath{37} = 'robot_state.multi_dof_joint_state.transforms.rotation.w';
info.MatPath{38} = 'robot_state.multi_dof_joint_state.twist';
info.MatPath{39} = 'robot_state.multi_dof_joint_state.twist.linear';
info.MatPath{40} = 'robot_state.multi_dof_joint_state.twist.linear.x';
info.MatPath{41} = 'robot_state.multi_dof_joint_state.twist.linear.y';
info.MatPath{42} = 'robot_state.multi_dof_joint_state.twist.linear.z';
info.MatPath{43} = 'robot_state.multi_dof_joint_state.twist.angular';
info.MatPath{44} = 'robot_state.multi_dof_joint_state.twist.angular.x';
info.MatPath{45} = 'robot_state.multi_dof_joint_state.twist.angular.y';
info.MatPath{46} = 'robot_state.multi_dof_joint_state.twist.angular.z';
info.MatPath{47} = 'robot_state.multi_dof_joint_state.wrench';
info.MatPath{48} = 'robot_state.multi_dof_joint_state.wrench.force';
info.MatPath{49} = 'robot_state.multi_dof_joint_state.wrench.force.x';
info.MatPath{50} = 'robot_state.multi_dof_joint_state.wrench.force.y';
info.MatPath{51} = 'robot_state.multi_dof_joint_state.wrench.force.z';
info.MatPath{52} = 'robot_state.multi_dof_joint_state.wrench.torque';
info.MatPath{53} = 'robot_state.multi_dof_joint_state.wrench.torque.x';
info.MatPath{54} = 'robot_state.multi_dof_joint_state.wrench.torque.y';
info.MatPath{55} = 'robot_state.multi_dof_joint_state.wrench.torque.z';
info.MatPath{56} = 'robot_state.attached_collision_objects';
info.MatPath{57} = 'robot_state.attached_collision_objects.link_name';
info.MatPath{58} = 'robot_state.attached_collision_objects.object';
info.MatPath{59} = 'robot_state.attached_collision_objects.object.header';
info.MatPath{60} = 'robot_state.attached_collision_objects.object.header.seq';
info.MatPath{61} = 'robot_state.attached_collision_objects.object.header.stamp';
info.MatPath{62} = 'robot_state.attached_collision_objects.object.header.stamp.sec';
info.MatPath{63} = 'robot_state.attached_collision_objects.object.header.stamp.nsec';
info.MatPath{64} = 'robot_state.attached_collision_objects.object.header.frame_id';
info.MatPath{65} = 'robot_state.attached_collision_objects.object.pose';
info.MatPath{66} = 'robot_state.attached_collision_objects.object.pose.position';
info.MatPath{67} = 'robot_state.attached_collision_objects.object.pose.position.x';
info.MatPath{68} = 'robot_state.attached_collision_objects.object.pose.position.y';
info.MatPath{69} = 'robot_state.attached_collision_objects.object.pose.position.z';
info.MatPath{70} = 'robot_state.attached_collision_objects.object.pose.orientation';
info.MatPath{71} = 'robot_state.attached_collision_objects.object.pose.orientation.x';
info.MatPath{72} = 'robot_state.attached_collision_objects.object.pose.orientation.y';
info.MatPath{73} = 'robot_state.attached_collision_objects.object.pose.orientation.z';
info.MatPath{74} = 'robot_state.attached_collision_objects.object.pose.orientation.w';
info.MatPath{75} = 'robot_state.attached_collision_objects.object.id';
info.MatPath{76} = 'robot_state.attached_collision_objects.object.type';
info.MatPath{77} = 'robot_state.attached_collision_objects.object.type.key';
info.MatPath{78} = 'robot_state.attached_collision_objects.object.type.db';
info.MatPath{79} = 'robot_state.attached_collision_objects.object.primitives';
info.MatPath{80} = 'robot_state.attached_collision_objects.object.primitives.BOX';
info.MatPath{81} = 'robot_state.attached_collision_objects.object.primitives.SPHERE';
info.MatPath{82} = 'robot_state.attached_collision_objects.object.primitives.CYLINDER';
info.MatPath{83} = 'robot_state.attached_collision_objects.object.primitives.CONE';
info.MatPath{84} = 'robot_state.attached_collision_objects.object.primitives.type';
info.MatPath{85} = 'robot_state.attached_collision_objects.object.primitives.dimensions';
info.MatPath{86} = 'robot_state.attached_collision_objects.object.primitives.BOX_X';
info.MatPath{87} = 'robot_state.attached_collision_objects.object.primitives.BOX_Y';
info.MatPath{88} = 'robot_state.attached_collision_objects.object.primitives.BOX_Z';
info.MatPath{89} = 'robot_state.attached_collision_objects.object.primitives.SPHERE_RADIUS';
info.MatPath{90} = 'robot_state.attached_collision_objects.object.primitives.CYLINDER_HEIGHT';
info.MatPath{91} = 'robot_state.attached_collision_objects.object.primitives.CYLINDER_RADIUS';
info.MatPath{92} = 'robot_state.attached_collision_objects.object.primitives.CONE_HEIGHT';
info.MatPath{93} = 'robot_state.attached_collision_objects.object.primitives.CONE_RADIUS';
info.MatPath{94} = 'robot_state.attached_collision_objects.object.primitive_poses';
info.MatPath{95} = 'robot_state.attached_collision_objects.object.primitive_poses.position';
info.MatPath{96} = 'robot_state.attached_collision_objects.object.primitive_poses.position.x';
info.MatPath{97} = 'robot_state.attached_collision_objects.object.primitive_poses.position.y';
info.MatPath{98} = 'robot_state.attached_collision_objects.object.primitive_poses.position.z';
info.MatPath{99} = 'robot_state.attached_collision_objects.object.primitive_poses.orientation';
info.MatPath{100} = 'robot_state.attached_collision_objects.object.primitive_poses.orientation.x';
info.MatPath{101} = 'robot_state.attached_collision_objects.object.primitive_poses.orientation.y';
info.MatPath{102} = 'robot_state.attached_collision_objects.object.primitive_poses.orientation.z';
info.MatPath{103} = 'robot_state.attached_collision_objects.object.primitive_poses.orientation.w';
info.MatPath{104} = 'robot_state.attached_collision_objects.object.meshes';
info.MatPath{105} = 'robot_state.attached_collision_objects.object.meshes.triangles';
info.MatPath{106} = 'robot_state.attached_collision_objects.object.meshes.triangles.vertex_indices';
info.MatPath{107} = 'robot_state.attached_collision_objects.object.meshes.vertices';
info.MatPath{108} = 'robot_state.attached_collision_objects.object.meshes.vertices.x';
info.MatPath{109} = 'robot_state.attached_collision_objects.object.meshes.vertices.y';
info.MatPath{110} = 'robot_state.attached_collision_objects.object.meshes.vertices.z';
info.MatPath{111} = 'robot_state.attached_collision_objects.object.mesh_poses';
info.MatPath{112} = 'robot_state.attached_collision_objects.object.mesh_poses.position';
info.MatPath{113} = 'robot_state.attached_collision_objects.object.mesh_poses.position.x';
info.MatPath{114} = 'robot_state.attached_collision_objects.object.mesh_poses.position.y';
info.MatPath{115} = 'robot_state.attached_collision_objects.object.mesh_poses.position.z';
info.MatPath{116} = 'robot_state.attached_collision_objects.object.mesh_poses.orientation';
info.MatPath{117} = 'robot_state.attached_collision_objects.object.mesh_poses.orientation.x';
info.MatPath{118} = 'robot_state.attached_collision_objects.object.mesh_poses.orientation.y';
info.MatPath{119} = 'robot_state.attached_collision_objects.object.mesh_poses.orientation.z';
info.MatPath{120} = 'robot_state.attached_collision_objects.object.mesh_poses.orientation.w';
info.MatPath{121} = 'robot_state.attached_collision_objects.object.planes';
info.MatPath{122} = 'robot_state.attached_collision_objects.object.planes.coef';
info.MatPath{123} = 'robot_state.attached_collision_objects.object.plane_poses';
info.MatPath{124} = 'robot_state.attached_collision_objects.object.plane_poses.position';
info.MatPath{125} = 'robot_state.attached_collision_objects.object.plane_poses.position.x';
info.MatPath{126} = 'robot_state.attached_collision_objects.object.plane_poses.position.y';
info.MatPath{127} = 'robot_state.attached_collision_objects.object.plane_poses.position.z';
info.MatPath{128} = 'robot_state.attached_collision_objects.object.plane_poses.orientation';
info.MatPath{129} = 'robot_state.attached_collision_objects.object.plane_poses.orientation.x';
info.MatPath{130} = 'robot_state.attached_collision_objects.object.plane_poses.orientation.y';
info.MatPath{131} = 'robot_state.attached_collision_objects.object.plane_poses.orientation.z';
info.MatPath{132} = 'robot_state.attached_collision_objects.object.plane_poses.orientation.w';
info.MatPath{133} = 'robot_state.attached_collision_objects.object.subframe_names';
info.MatPath{134} = 'robot_state.attached_collision_objects.object.subframe_poses';
info.MatPath{135} = 'robot_state.attached_collision_objects.object.subframe_poses.position';
info.MatPath{136} = 'robot_state.attached_collision_objects.object.subframe_poses.position.x';
info.MatPath{137} = 'robot_state.attached_collision_objects.object.subframe_poses.position.y';
info.MatPath{138} = 'robot_state.attached_collision_objects.object.subframe_poses.position.z';
info.MatPath{139} = 'robot_state.attached_collision_objects.object.subframe_poses.orientation';
info.MatPath{140} = 'robot_state.attached_collision_objects.object.subframe_poses.orientation.x';
info.MatPath{141} = 'robot_state.attached_collision_objects.object.subframe_poses.orientation.y';
info.MatPath{142} = 'robot_state.attached_collision_objects.object.subframe_poses.orientation.z';
info.MatPath{143} = 'robot_state.attached_collision_objects.object.subframe_poses.orientation.w';
info.MatPath{144} = 'robot_state.attached_collision_objects.object.ADD';
info.MatPath{145} = 'robot_state.attached_collision_objects.object.REMOVE';
info.MatPath{146} = 'robot_state.attached_collision_objects.object.APPEND';
info.MatPath{147} = 'robot_state.attached_collision_objects.object.MOVE';
info.MatPath{148} = 'robot_state.attached_collision_objects.object.operation';
info.MatPath{149} = 'robot_state.attached_collision_objects.touch_links';
info.MatPath{150} = 'robot_state.attached_collision_objects.detach_posture';
info.MatPath{151} = 'robot_state.attached_collision_objects.detach_posture.header';
info.MatPath{152} = 'robot_state.attached_collision_objects.detach_posture.header.seq';
info.MatPath{153} = 'robot_state.attached_collision_objects.detach_posture.header.stamp';
info.MatPath{154} = 'robot_state.attached_collision_objects.detach_posture.header.stamp.sec';
info.MatPath{155} = 'robot_state.attached_collision_objects.detach_posture.header.stamp.nsec';
info.MatPath{156} = 'robot_state.attached_collision_objects.detach_posture.header.frame_id';
info.MatPath{157} = 'robot_state.attached_collision_objects.detach_posture.joint_names';
info.MatPath{158} = 'robot_state.attached_collision_objects.detach_posture.points';
info.MatPath{159} = 'robot_state.attached_collision_objects.detach_posture.points.positions';
info.MatPath{160} = 'robot_state.attached_collision_objects.detach_posture.points.velocities';
info.MatPath{161} = 'robot_state.attached_collision_objects.detach_posture.points.accelerations';
info.MatPath{162} = 'robot_state.attached_collision_objects.detach_posture.points.effort';
info.MatPath{163} = 'robot_state.attached_collision_objects.detach_posture.points.time_from_start';
info.MatPath{164} = 'robot_state.attached_collision_objects.detach_posture.points.time_from_start.sec';
info.MatPath{165} = 'robot_state.attached_collision_objects.detach_posture.points.time_from_start.nsec';
info.MatPath{166} = 'robot_state.attached_collision_objects.weight';
info.MatPath{167} = 'robot_state.is_diff';