function [data, info] = planningScene
%PlanningScene gives an empty data for moveit_msgs/PlanningScene

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'moveit_msgs/PlanningScene';
[data.Name, info.Name] = ros.internal.ros.messages.ros.char('string',0);
[data.RobotState, info.RobotState] = ros.internal.ros.messages.moveit_msgs.robotState;
info.RobotState.MLdataType = 'struct';
[data.RobotModelName, info.RobotModelName] = ros.internal.ros.messages.ros.char('string',0);
[data.FixedFrameTransforms, info.FixedFrameTransforms] = ros.internal.ros.messages.geometry_msgs.transformStamped;
info.FixedFrameTransforms.MLdataType = 'struct';
info.FixedFrameTransforms.MaxLen = NaN;
info.FixedFrameTransforms.MinLen = 0;
data.FixedFrameTransforms = data.FixedFrameTransforms([],1);
[data.AllowedCollisionMatrix, info.AllowedCollisionMatrix] = ros.internal.ros.messages.moveit_msgs.allowedCollisionMatrix;
info.AllowedCollisionMatrix.MLdataType = 'struct';
[data.LinkPadding, info.LinkPadding] = ros.internal.ros.messages.moveit_msgs.linkPadding;
info.LinkPadding.MLdataType = 'struct';
info.LinkPadding.MaxLen = NaN;
info.LinkPadding.MinLen = 0;
data.LinkPadding = data.LinkPadding([],1);
[data.LinkScale, info.LinkScale] = ros.internal.ros.messages.moveit_msgs.linkScale;
info.LinkScale.MLdataType = 'struct';
info.LinkScale.MaxLen = NaN;
info.LinkScale.MinLen = 0;
data.LinkScale = data.LinkScale([],1);
[data.ObjectColors, info.ObjectColors] = ros.internal.ros.messages.moveit_msgs.objectColor;
info.ObjectColors.MLdataType = 'struct';
info.ObjectColors.MaxLen = NaN;
info.ObjectColors.MinLen = 0;
data.ObjectColors = data.ObjectColors([],1);
[data.World, info.World] = ros.internal.ros.messages.moveit_msgs.planningSceneWorld;
info.World.MLdataType = 'struct';
[data.IsDiff, info.IsDiff] = ros.internal.ros.messages.ros.default_type('logical',1);
info.MessageType = 'moveit_msgs/PlanningScene';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,320);
info.MatPath{1} = 'name';
info.MatPath{2} = 'robot_state';
info.MatPath{3} = 'robot_state.joint_state';
info.MatPath{4} = 'robot_state.joint_state.header';
info.MatPath{5} = 'robot_state.joint_state.header.seq';
info.MatPath{6} = 'robot_state.joint_state.header.stamp';
info.MatPath{7} = 'robot_state.joint_state.header.stamp.sec';
info.MatPath{8} = 'robot_state.joint_state.header.stamp.nsec';
info.MatPath{9} = 'robot_state.joint_state.header.frame_id';
info.MatPath{10} = 'robot_state.joint_state.name';
info.MatPath{11} = 'robot_state.joint_state.position';
info.MatPath{12} = 'robot_state.joint_state.velocity';
info.MatPath{13} = 'robot_state.joint_state.effort';
info.MatPath{14} = 'robot_state.multi_dof_joint_state';
info.MatPath{15} = 'robot_state.multi_dof_joint_state.header';
info.MatPath{16} = 'robot_state.multi_dof_joint_state.header.seq';
info.MatPath{17} = 'robot_state.multi_dof_joint_state.header.stamp';
info.MatPath{18} = 'robot_state.multi_dof_joint_state.header.stamp.sec';
info.MatPath{19} = 'robot_state.multi_dof_joint_state.header.stamp.nsec';
info.MatPath{20} = 'robot_state.multi_dof_joint_state.header.frame_id';
info.MatPath{21} = 'robot_state.multi_dof_joint_state.joint_names';
info.MatPath{22} = 'robot_state.multi_dof_joint_state.transforms';
info.MatPath{23} = 'robot_state.multi_dof_joint_state.transforms.translation';
info.MatPath{24} = 'robot_state.multi_dof_joint_state.transforms.translation.x';
info.MatPath{25} = 'robot_state.multi_dof_joint_state.transforms.translation.y';
info.MatPath{26} = 'robot_state.multi_dof_joint_state.transforms.translation.z';
info.MatPath{27} = 'robot_state.multi_dof_joint_state.transforms.rotation';
info.MatPath{28} = 'robot_state.multi_dof_joint_state.transforms.rotation.x';
info.MatPath{29} = 'robot_state.multi_dof_joint_state.transforms.rotation.y';
info.MatPath{30} = 'robot_state.multi_dof_joint_state.transforms.rotation.z';
info.MatPath{31} = 'robot_state.multi_dof_joint_state.transforms.rotation.w';
info.MatPath{32} = 'robot_state.multi_dof_joint_state.twist';
info.MatPath{33} = 'robot_state.multi_dof_joint_state.twist.linear';
info.MatPath{34} = 'robot_state.multi_dof_joint_state.twist.linear.x';
info.MatPath{35} = 'robot_state.multi_dof_joint_state.twist.linear.y';
info.MatPath{36} = 'robot_state.multi_dof_joint_state.twist.linear.z';
info.MatPath{37} = 'robot_state.multi_dof_joint_state.twist.angular';
info.MatPath{38} = 'robot_state.multi_dof_joint_state.twist.angular.x';
info.MatPath{39} = 'robot_state.multi_dof_joint_state.twist.angular.y';
info.MatPath{40} = 'robot_state.multi_dof_joint_state.twist.angular.z';
info.MatPath{41} = 'robot_state.multi_dof_joint_state.wrench';
info.MatPath{42} = 'robot_state.multi_dof_joint_state.wrench.force';
info.MatPath{43} = 'robot_state.multi_dof_joint_state.wrench.force.x';
info.MatPath{44} = 'robot_state.multi_dof_joint_state.wrench.force.y';
info.MatPath{45} = 'robot_state.multi_dof_joint_state.wrench.force.z';
info.MatPath{46} = 'robot_state.multi_dof_joint_state.wrench.torque';
info.MatPath{47} = 'robot_state.multi_dof_joint_state.wrench.torque.x';
info.MatPath{48} = 'robot_state.multi_dof_joint_state.wrench.torque.y';
info.MatPath{49} = 'robot_state.multi_dof_joint_state.wrench.torque.z';
info.MatPath{50} = 'robot_state.attached_collision_objects';
info.MatPath{51} = 'robot_state.attached_collision_objects.link_name';
info.MatPath{52} = 'robot_state.attached_collision_objects.object';
info.MatPath{53} = 'robot_state.attached_collision_objects.object.header';
info.MatPath{54} = 'robot_state.attached_collision_objects.object.header.seq';
info.MatPath{55} = 'robot_state.attached_collision_objects.object.header.stamp';
info.MatPath{56} = 'robot_state.attached_collision_objects.object.header.stamp.sec';
info.MatPath{57} = 'robot_state.attached_collision_objects.object.header.stamp.nsec';
info.MatPath{58} = 'robot_state.attached_collision_objects.object.header.frame_id';
info.MatPath{59} = 'robot_state.attached_collision_objects.object.pose';
info.MatPath{60} = 'robot_state.attached_collision_objects.object.pose.position';
info.MatPath{61} = 'robot_state.attached_collision_objects.object.pose.position.x';
info.MatPath{62} = 'robot_state.attached_collision_objects.object.pose.position.y';
info.MatPath{63} = 'robot_state.attached_collision_objects.object.pose.position.z';
info.MatPath{64} = 'robot_state.attached_collision_objects.object.pose.orientation';
info.MatPath{65} = 'robot_state.attached_collision_objects.object.pose.orientation.x';
info.MatPath{66} = 'robot_state.attached_collision_objects.object.pose.orientation.y';
info.MatPath{67} = 'robot_state.attached_collision_objects.object.pose.orientation.z';
info.MatPath{68} = 'robot_state.attached_collision_objects.object.pose.orientation.w';
info.MatPath{69} = 'robot_state.attached_collision_objects.object.id';
info.MatPath{70} = 'robot_state.attached_collision_objects.object.type';
info.MatPath{71} = 'robot_state.attached_collision_objects.object.type.key';
info.MatPath{72} = 'robot_state.attached_collision_objects.object.type.db';
info.MatPath{73} = 'robot_state.attached_collision_objects.object.primitives';
info.MatPath{74} = 'robot_state.attached_collision_objects.object.primitives.BOX';
info.MatPath{75} = 'robot_state.attached_collision_objects.object.primitives.SPHERE';
info.MatPath{76} = 'robot_state.attached_collision_objects.object.primitives.CYLINDER';
info.MatPath{77} = 'robot_state.attached_collision_objects.object.primitives.CONE';
info.MatPath{78} = 'robot_state.attached_collision_objects.object.primitives.type';
info.MatPath{79} = 'robot_state.attached_collision_objects.object.primitives.dimensions';
info.MatPath{80} = 'robot_state.attached_collision_objects.object.primitives.BOX_X';
info.MatPath{81} = 'robot_state.attached_collision_objects.object.primitives.BOX_Y';
info.MatPath{82} = 'robot_state.attached_collision_objects.object.primitives.BOX_Z';
info.MatPath{83} = 'robot_state.attached_collision_objects.object.primitives.SPHERE_RADIUS';
info.MatPath{84} = 'robot_state.attached_collision_objects.object.primitives.CYLINDER_HEIGHT';
info.MatPath{85} = 'robot_state.attached_collision_objects.object.primitives.CYLINDER_RADIUS';
info.MatPath{86} = 'robot_state.attached_collision_objects.object.primitives.CONE_HEIGHT';
info.MatPath{87} = 'robot_state.attached_collision_objects.object.primitives.CONE_RADIUS';
info.MatPath{88} = 'robot_state.attached_collision_objects.object.primitive_poses';
info.MatPath{89} = 'robot_state.attached_collision_objects.object.primitive_poses.position';
info.MatPath{90} = 'robot_state.attached_collision_objects.object.primitive_poses.position.x';
info.MatPath{91} = 'robot_state.attached_collision_objects.object.primitive_poses.position.y';
info.MatPath{92} = 'robot_state.attached_collision_objects.object.primitive_poses.position.z';
info.MatPath{93} = 'robot_state.attached_collision_objects.object.primitive_poses.orientation';
info.MatPath{94} = 'robot_state.attached_collision_objects.object.primitive_poses.orientation.x';
info.MatPath{95} = 'robot_state.attached_collision_objects.object.primitive_poses.orientation.y';
info.MatPath{96} = 'robot_state.attached_collision_objects.object.primitive_poses.orientation.z';
info.MatPath{97} = 'robot_state.attached_collision_objects.object.primitive_poses.orientation.w';
info.MatPath{98} = 'robot_state.attached_collision_objects.object.meshes';
info.MatPath{99} = 'robot_state.attached_collision_objects.object.meshes.triangles';
info.MatPath{100} = 'robot_state.attached_collision_objects.object.meshes.triangles.vertex_indices';
info.MatPath{101} = 'robot_state.attached_collision_objects.object.meshes.vertices';
info.MatPath{102} = 'robot_state.attached_collision_objects.object.meshes.vertices.x';
info.MatPath{103} = 'robot_state.attached_collision_objects.object.meshes.vertices.y';
info.MatPath{104} = 'robot_state.attached_collision_objects.object.meshes.vertices.z';
info.MatPath{105} = 'robot_state.attached_collision_objects.object.mesh_poses';
info.MatPath{106} = 'robot_state.attached_collision_objects.object.mesh_poses.position';
info.MatPath{107} = 'robot_state.attached_collision_objects.object.mesh_poses.position.x';
info.MatPath{108} = 'robot_state.attached_collision_objects.object.mesh_poses.position.y';
info.MatPath{109} = 'robot_state.attached_collision_objects.object.mesh_poses.position.z';
info.MatPath{110} = 'robot_state.attached_collision_objects.object.mesh_poses.orientation';
info.MatPath{111} = 'robot_state.attached_collision_objects.object.mesh_poses.orientation.x';
info.MatPath{112} = 'robot_state.attached_collision_objects.object.mesh_poses.orientation.y';
info.MatPath{113} = 'robot_state.attached_collision_objects.object.mesh_poses.orientation.z';
info.MatPath{114} = 'robot_state.attached_collision_objects.object.mesh_poses.orientation.w';
info.MatPath{115} = 'robot_state.attached_collision_objects.object.planes';
info.MatPath{116} = 'robot_state.attached_collision_objects.object.planes.coef';
info.MatPath{117} = 'robot_state.attached_collision_objects.object.plane_poses';
info.MatPath{118} = 'robot_state.attached_collision_objects.object.plane_poses.position';
info.MatPath{119} = 'robot_state.attached_collision_objects.object.plane_poses.position.x';
info.MatPath{120} = 'robot_state.attached_collision_objects.object.plane_poses.position.y';
info.MatPath{121} = 'robot_state.attached_collision_objects.object.plane_poses.position.z';
info.MatPath{122} = 'robot_state.attached_collision_objects.object.plane_poses.orientation';
info.MatPath{123} = 'robot_state.attached_collision_objects.object.plane_poses.orientation.x';
info.MatPath{124} = 'robot_state.attached_collision_objects.object.plane_poses.orientation.y';
info.MatPath{125} = 'robot_state.attached_collision_objects.object.plane_poses.orientation.z';
info.MatPath{126} = 'robot_state.attached_collision_objects.object.plane_poses.orientation.w';
info.MatPath{127} = 'robot_state.attached_collision_objects.object.subframe_names';
info.MatPath{128} = 'robot_state.attached_collision_objects.object.subframe_poses';
info.MatPath{129} = 'robot_state.attached_collision_objects.object.subframe_poses.position';
info.MatPath{130} = 'robot_state.attached_collision_objects.object.subframe_poses.position.x';
info.MatPath{131} = 'robot_state.attached_collision_objects.object.subframe_poses.position.y';
info.MatPath{132} = 'robot_state.attached_collision_objects.object.subframe_poses.position.z';
info.MatPath{133} = 'robot_state.attached_collision_objects.object.subframe_poses.orientation';
info.MatPath{134} = 'robot_state.attached_collision_objects.object.subframe_poses.orientation.x';
info.MatPath{135} = 'robot_state.attached_collision_objects.object.subframe_poses.orientation.y';
info.MatPath{136} = 'robot_state.attached_collision_objects.object.subframe_poses.orientation.z';
info.MatPath{137} = 'robot_state.attached_collision_objects.object.subframe_poses.orientation.w';
info.MatPath{138} = 'robot_state.attached_collision_objects.object.ADD';
info.MatPath{139} = 'robot_state.attached_collision_objects.object.REMOVE';
info.MatPath{140} = 'robot_state.attached_collision_objects.object.APPEND';
info.MatPath{141} = 'robot_state.attached_collision_objects.object.MOVE';
info.MatPath{142} = 'robot_state.attached_collision_objects.object.operation';
info.MatPath{143} = 'robot_state.attached_collision_objects.touch_links';
info.MatPath{144} = 'robot_state.attached_collision_objects.detach_posture';
info.MatPath{145} = 'robot_state.attached_collision_objects.detach_posture.header';
info.MatPath{146} = 'robot_state.attached_collision_objects.detach_posture.header.seq';
info.MatPath{147} = 'robot_state.attached_collision_objects.detach_posture.header.stamp';
info.MatPath{148} = 'robot_state.attached_collision_objects.detach_posture.header.stamp.sec';
info.MatPath{149} = 'robot_state.attached_collision_objects.detach_posture.header.stamp.nsec';
info.MatPath{150} = 'robot_state.attached_collision_objects.detach_posture.header.frame_id';
info.MatPath{151} = 'robot_state.attached_collision_objects.detach_posture.joint_names';
info.MatPath{152} = 'robot_state.attached_collision_objects.detach_posture.points';
info.MatPath{153} = 'robot_state.attached_collision_objects.detach_posture.points.positions';
info.MatPath{154} = 'robot_state.attached_collision_objects.detach_posture.points.velocities';
info.MatPath{155} = 'robot_state.attached_collision_objects.detach_posture.points.accelerations';
info.MatPath{156} = 'robot_state.attached_collision_objects.detach_posture.points.effort';
info.MatPath{157} = 'robot_state.attached_collision_objects.detach_posture.points.time_from_start';
info.MatPath{158} = 'robot_state.attached_collision_objects.detach_posture.points.time_from_start.sec';
info.MatPath{159} = 'robot_state.attached_collision_objects.detach_posture.points.time_from_start.nsec';
info.MatPath{160} = 'robot_state.attached_collision_objects.weight';
info.MatPath{161} = 'robot_state.is_diff';
info.MatPath{162} = 'robot_model_name';
info.MatPath{163} = 'fixed_frame_transforms';
info.MatPath{164} = 'fixed_frame_transforms.header';
info.MatPath{165} = 'fixed_frame_transforms.header.seq';
info.MatPath{166} = 'fixed_frame_transforms.header.stamp';
info.MatPath{167} = 'fixed_frame_transforms.header.stamp.sec';
info.MatPath{168} = 'fixed_frame_transforms.header.stamp.nsec';
info.MatPath{169} = 'fixed_frame_transforms.header.frame_id';
info.MatPath{170} = 'fixed_frame_transforms.child_frame_id';
info.MatPath{171} = 'fixed_frame_transforms.transform';
info.MatPath{172} = 'fixed_frame_transforms.transform.translation';
info.MatPath{173} = 'fixed_frame_transforms.transform.translation.x';
info.MatPath{174} = 'fixed_frame_transforms.transform.translation.y';
info.MatPath{175} = 'fixed_frame_transforms.transform.translation.z';
info.MatPath{176} = 'fixed_frame_transforms.transform.rotation';
info.MatPath{177} = 'fixed_frame_transforms.transform.rotation.x';
info.MatPath{178} = 'fixed_frame_transforms.transform.rotation.y';
info.MatPath{179} = 'fixed_frame_transforms.transform.rotation.z';
info.MatPath{180} = 'fixed_frame_transforms.transform.rotation.w';
info.MatPath{181} = 'allowed_collision_matrix';
info.MatPath{182} = 'allowed_collision_matrix.entry_names';
info.MatPath{183} = 'allowed_collision_matrix.entry_values';
info.MatPath{184} = 'allowed_collision_matrix.entry_values.enabled';
info.MatPath{185} = 'allowed_collision_matrix.default_entry_names';
info.MatPath{186} = 'allowed_collision_matrix.default_entry_values';
info.MatPath{187} = 'link_padding';
info.MatPath{188} = 'link_padding.link_name';
info.MatPath{189} = 'link_padding.padding';
info.MatPath{190} = 'link_scale';
info.MatPath{191} = 'link_scale.link_name';
info.MatPath{192} = 'link_scale.scale';
info.MatPath{193} = 'object_colors';
info.MatPath{194} = 'object_colors.id';
info.MatPath{195} = 'object_colors.color';
info.MatPath{196} = 'object_colors.color.r';
info.MatPath{197} = 'object_colors.color.g';
info.MatPath{198} = 'object_colors.color.b';
info.MatPath{199} = 'object_colors.color.a';
info.MatPath{200} = 'world';
info.MatPath{201} = 'world.collision_objects';
info.MatPath{202} = 'world.collision_objects.header';
info.MatPath{203} = 'world.collision_objects.header.seq';
info.MatPath{204} = 'world.collision_objects.header.stamp';
info.MatPath{205} = 'world.collision_objects.header.stamp.sec';
info.MatPath{206} = 'world.collision_objects.header.stamp.nsec';
info.MatPath{207} = 'world.collision_objects.header.frame_id';
info.MatPath{208} = 'world.collision_objects.pose';
info.MatPath{209} = 'world.collision_objects.pose.position';
info.MatPath{210} = 'world.collision_objects.pose.position.x';
info.MatPath{211} = 'world.collision_objects.pose.position.y';
info.MatPath{212} = 'world.collision_objects.pose.position.z';
info.MatPath{213} = 'world.collision_objects.pose.orientation';
info.MatPath{214} = 'world.collision_objects.pose.orientation.x';
info.MatPath{215} = 'world.collision_objects.pose.orientation.y';
info.MatPath{216} = 'world.collision_objects.pose.orientation.z';
info.MatPath{217} = 'world.collision_objects.pose.orientation.w';
info.MatPath{218} = 'world.collision_objects.id';
info.MatPath{219} = 'world.collision_objects.type';
info.MatPath{220} = 'world.collision_objects.type.key';
info.MatPath{221} = 'world.collision_objects.type.db';
info.MatPath{222} = 'world.collision_objects.primitives';
info.MatPath{223} = 'world.collision_objects.primitives.BOX';
info.MatPath{224} = 'world.collision_objects.primitives.SPHERE';
info.MatPath{225} = 'world.collision_objects.primitives.CYLINDER';
info.MatPath{226} = 'world.collision_objects.primitives.CONE';
info.MatPath{227} = 'world.collision_objects.primitives.type';
info.MatPath{228} = 'world.collision_objects.primitives.dimensions';
info.MatPath{229} = 'world.collision_objects.primitives.BOX_X';
info.MatPath{230} = 'world.collision_objects.primitives.BOX_Y';
info.MatPath{231} = 'world.collision_objects.primitives.BOX_Z';
info.MatPath{232} = 'world.collision_objects.primitives.SPHERE_RADIUS';
info.MatPath{233} = 'world.collision_objects.primitives.CYLINDER_HEIGHT';
info.MatPath{234} = 'world.collision_objects.primitives.CYLINDER_RADIUS';
info.MatPath{235} = 'world.collision_objects.primitives.CONE_HEIGHT';
info.MatPath{236} = 'world.collision_objects.primitives.CONE_RADIUS';
info.MatPath{237} = 'world.collision_objects.primitive_poses';
info.MatPath{238} = 'world.collision_objects.primitive_poses.position';
info.MatPath{239} = 'world.collision_objects.primitive_poses.position.x';
info.MatPath{240} = 'world.collision_objects.primitive_poses.position.y';
info.MatPath{241} = 'world.collision_objects.primitive_poses.position.z';
info.MatPath{242} = 'world.collision_objects.primitive_poses.orientation';
info.MatPath{243} = 'world.collision_objects.primitive_poses.orientation.x';
info.MatPath{244} = 'world.collision_objects.primitive_poses.orientation.y';
info.MatPath{245} = 'world.collision_objects.primitive_poses.orientation.z';
info.MatPath{246} = 'world.collision_objects.primitive_poses.orientation.w';
info.MatPath{247} = 'world.collision_objects.meshes';
info.MatPath{248} = 'world.collision_objects.meshes.triangles';
info.MatPath{249} = 'world.collision_objects.meshes.triangles.vertex_indices';
info.MatPath{250} = 'world.collision_objects.meshes.vertices';
info.MatPath{251} = 'world.collision_objects.meshes.vertices.x';
info.MatPath{252} = 'world.collision_objects.meshes.vertices.y';
info.MatPath{253} = 'world.collision_objects.meshes.vertices.z';
info.MatPath{254} = 'world.collision_objects.mesh_poses';
info.MatPath{255} = 'world.collision_objects.mesh_poses.position';
info.MatPath{256} = 'world.collision_objects.mesh_poses.position.x';
info.MatPath{257} = 'world.collision_objects.mesh_poses.position.y';
info.MatPath{258} = 'world.collision_objects.mesh_poses.position.z';
info.MatPath{259} = 'world.collision_objects.mesh_poses.orientation';
info.MatPath{260} = 'world.collision_objects.mesh_poses.orientation.x';
info.MatPath{261} = 'world.collision_objects.mesh_poses.orientation.y';
info.MatPath{262} = 'world.collision_objects.mesh_poses.orientation.z';
info.MatPath{263} = 'world.collision_objects.mesh_poses.orientation.w';
info.MatPath{264} = 'world.collision_objects.planes';
info.MatPath{265} = 'world.collision_objects.planes.coef';
info.MatPath{266} = 'world.collision_objects.plane_poses';
info.MatPath{267} = 'world.collision_objects.plane_poses.position';
info.MatPath{268} = 'world.collision_objects.plane_poses.position.x';
info.MatPath{269} = 'world.collision_objects.plane_poses.position.y';
info.MatPath{270} = 'world.collision_objects.plane_poses.position.z';
info.MatPath{271} = 'world.collision_objects.plane_poses.orientation';
info.MatPath{272} = 'world.collision_objects.plane_poses.orientation.x';
info.MatPath{273} = 'world.collision_objects.plane_poses.orientation.y';
info.MatPath{274} = 'world.collision_objects.plane_poses.orientation.z';
info.MatPath{275} = 'world.collision_objects.plane_poses.orientation.w';
info.MatPath{276} = 'world.collision_objects.subframe_names';
info.MatPath{277} = 'world.collision_objects.subframe_poses';
info.MatPath{278} = 'world.collision_objects.subframe_poses.position';
info.MatPath{279} = 'world.collision_objects.subframe_poses.position.x';
info.MatPath{280} = 'world.collision_objects.subframe_poses.position.y';
info.MatPath{281} = 'world.collision_objects.subframe_poses.position.z';
info.MatPath{282} = 'world.collision_objects.subframe_poses.orientation';
info.MatPath{283} = 'world.collision_objects.subframe_poses.orientation.x';
info.MatPath{284} = 'world.collision_objects.subframe_poses.orientation.y';
info.MatPath{285} = 'world.collision_objects.subframe_poses.orientation.z';
info.MatPath{286} = 'world.collision_objects.subframe_poses.orientation.w';
info.MatPath{287} = 'world.collision_objects.ADD';
info.MatPath{288} = 'world.collision_objects.REMOVE';
info.MatPath{289} = 'world.collision_objects.APPEND';
info.MatPath{290} = 'world.collision_objects.MOVE';
info.MatPath{291} = 'world.collision_objects.operation';
info.MatPath{292} = 'world.octomap';
info.MatPath{293} = 'world.octomap.header';
info.MatPath{294} = 'world.octomap.header.seq';
info.MatPath{295} = 'world.octomap.header.stamp';
info.MatPath{296} = 'world.octomap.header.stamp.sec';
info.MatPath{297} = 'world.octomap.header.stamp.nsec';
info.MatPath{298} = 'world.octomap.header.frame_id';
info.MatPath{299} = 'world.octomap.origin';
info.MatPath{300} = 'world.octomap.origin.position';
info.MatPath{301} = 'world.octomap.origin.position.x';
info.MatPath{302} = 'world.octomap.origin.position.y';
info.MatPath{303} = 'world.octomap.origin.position.z';
info.MatPath{304} = 'world.octomap.origin.orientation';
info.MatPath{305} = 'world.octomap.origin.orientation.x';
info.MatPath{306} = 'world.octomap.origin.orientation.y';
info.MatPath{307} = 'world.octomap.origin.orientation.z';
info.MatPath{308} = 'world.octomap.origin.orientation.w';
info.MatPath{309} = 'world.octomap.octomap';
info.MatPath{310} = 'world.octomap.octomap.header';
info.MatPath{311} = 'world.octomap.octomap.header.seq';
info.MatPath{312} = 'world.octomap.octomap.header.stamp';
info.MatPath{313} = 'world.octomap.octomap.header.stamp.sec';
info.MatPath{314} = 'world.octomap.octomap.header.stamp.nsec';
info.MatPath{315} = 'world.octomap.octomap.header.frame_id';
info.MatPath{316} = 'world.octomap.octomap.binary';
info.MatPath{317} = 'world.octomap.octomap.id';
info.MatPath{318} = 'world.octomap.octomap.resolution';
info.MatPath{319} = 'world.octomap.octomap.data';
info.MatPath{320} = 'is_diff';