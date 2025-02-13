function [data, info] = getCartesianPathResponse
%GetCartesianPath gives an empty data for moveit_msgs/GetCartesianPathResponse

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'moveit_msgs/GetCartesianPathResponse';
[data.StartState, info.StartState] = ros.internal.ros.messages.moveit_msgs.robotState;
info.StartState.MLdataType = 'struct';
[data.Solution, info.Solution] = ros.internal.ros.messages.moveit_msgs.robotTrajectory;
info.Solution.MLdataType = 'struct';
[data.Fraction, info.Fraction] = ros.internal.ros.messages.ros.default_type('double',1);
[data.ErrorCode, info.ErrorCode] = ros.internal.ros.messages.moveit_msgs.moveItErrorCodes;
info.ErrorCode.MLdataType = 'struct';
info.MessageType = 'moveit_msgs/GetCartesianPathResponse';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,245);
info.MatPath{1} = 'start_state';
info.MatPath{2} = 'start_state.joint_state';
info.MatPath{3} = 'start_state.joint_state.header';
info.MatPath{4} = 'start_state.joint_state.header.seq';
info.MatPath{5} = 'start_state.joint_state.header.stamp';
info.MatPath{6} = 'start_state.joint_state.header.stamp.sec';
info.MatPath{7} = 'start_state.joint_state.header.stamp.nsec';
info.MatPath{8} = 'start_state.joint_state.header.frame_id';
info.MatPath{9} = 'start_state.joint_state.name';
info.MatPath{10} = 'start_state.joint_state.position';
info.MatPath{11} = 'start_state.joint_state.velocity';
info.MatPath{12} = 'start_state.joint_state.effort';
info.MatPath{13} = 'start_state.multi_dof_joint_state';
info.MatPath{14} = 'start_state.multi_dof_joint_state.header';
info.MatPath{15} = 'start_state.multi_dof_joint_state.header.seq';
info.MatPath{16} = 'start_state.multi_dof_joint_state.header.stamp';
info.MatPath{17} = 'start_state.multi_dof_joint_state.header.stamp.sec';
info.MatPath{18} = 'start_state.multi_dof_joint_state.header.stamp.nsec';
info.MatPath{19} = 'start_state.multi_dof_joint_state.header.frame_id';
info.MatPath{20} = 'start_state.multi_dof_joint_state.joint_names';
info.MatPath{21} = 'start_state.multi_dof_joint_state.transforms';
info.MatPath{22} = 'start_state.multi_dof_joint_state.transforms.translation';
info.MatPath{23} = 'start_state.multi_dof_joint_state.transforms.translation.x';
info.MatPath{24} = 'start_state.multi_dof_joint_state.transforms.translation.y';
info.MatPath{25} = 'start_state.multi_dof_joint_state.transforms.translation.z';
info.MatPath{26} = 'start_state.multi_dof_joint_state.transforms.rotation';
info.MatPath{27} = 'start_state.multi_dof_joint_state.transforms.rotation.x';
info.MatPath{28} = 'start_state.multi_dof_joint_state.transforms.rotation.y';
info.MatPath{29} = 'start_state.multi_dof_joint_state.transforms.rotation.z';
info.MatPath{30} = 'start_state.multi_dof_joint_state.transforms.rotation.w';
info.MatPath{31} = 'start_state.multi_dof_joint_state.twist';
info.MatPath{32} = 'start_state.multi_dof_joint_state.twist.linear';
info.MatPath{33} = 'start_state.multi_dof_joint_state.twist.linear.x';
info.MatPath{34} = 'start_state.multi_dof_joint_state.twist.linear.y';
info.MatPath{35} = 'start_state.multi_dof_joint_state.twist.linear.z';
info.MatPath{36} = 'start_state.multi_dof_joint_state.twist.angular';
info.MatPath{37} = 'start_state.multi_dof_joint_state.twist.angular.x';
info.MatPath{38} = 'start_state.multi_dof_joint_state.twist.angular.y';
info.MatPath{39} = 'start_state.multi_dof_joint_state.twist.angular.z';
info.MatPath{40} = 'start_state.multi_dof_joint_state.wrench';
info.MatPath{41} = 'start_state.multi_dof_joint_state.wrench.force';
info.MatPath{42} = 'start_state.multi_dof_joint_state.wrench.force.x';
info.MatPath{43} = 'start_state.multi_dof_joint_state.wrench.force.y';
info.MatPath{44} = 'start_state.multi_dof_joint_state.wrench.force.z';
info.MatPath{45} = 'start_state.multi_dof_joint_state.wrench.torque';
info.MatPath{46} = 'start_state.multi_dof_joint_state.wrench.torque.x';
info.MatPath{47} = 'start_state.multi_dof_joint_state.wrench.torque.y';
info.MatPath{48} = 'start_state.multi_dof_joint_state.wrench.torque.z';
info.MatPath{49} = 'start_state.attached_collision_objects';
info.MatPath{50} = 'start_state.attached_collision_objects.link_name';
info.MatPath{51} = 'start_state.attached_collision_objects.object';
info.MatPath{52} = 'start_state.attached_collision_objects.object.header';
info.MatPath{53} = 'start_state.attached_collision_objects.object.header.seq';
info.MatPath{54} = 'start_state.attached_collision_objects.object.header.stamp';
info.MatPath{55} = 'start_state.attached_collision_objects.object.header.stamp.sec';
info.MatPath{56} = 'start_state.attached_collision_objects.object.header.stamp.nsec';
info.MatPath{57} = 'start_state.attached_collision_objects.object.header.frame_id';
info.MatPath{58} = 'start_state.attached_collision_objects.object.pose';
info.MatPath{59} = 'start_state.attached_collision_objects.object.pose.position';
info.MatPath{60} = 'start_state.attached_collision_objects.object.pose.position.x';
info.MatPath{61} = 'start_state.attached_collision_objects.object.pose.position.y';
info.MatPath{62} = 'start_state.attached_collision_objects.object.pose.position.z';
info.MatPath{63} = 'start_state.attached_collision_objects.object.pose.orientation';
info.MatPath{64} = 'start_state.attached_collision_objects.object.pose.orientation.x';
info.MatPath{65} = 'start_state.attached_collision_objects.object.pose.orientation.y';
info.MatPath{66} = 'start_state.attached_collision_objects.object.pose.orientation.z';
info.MatPath{67} = 'start_state.attached_collision_objects.object.pose.orientation.w';
info.MatPath{68} = 'start_state.attached_collision_objects.object.id';
info.MatPath{69} = 'start_state.attached_collision_objects.object.type';
info.MatPath{70} = 'start_state.attached_collision_objects.object.type.key';
info.MatPath{71} = 'start_state.attached_collision_objects.object.type.db';
info.MatPath{72} = 'start_state.attached_collision_objects.object.primitives';
info.MatPath{73} = 'start_state.attached_collision_objects.object.primitives.BOX';
info.MatPath{74} = 'start_state.attached_collision_objects.object.primitives.SPHERE';
info.MatPath{75} = 'start_state.attached_collision_objects.object.primitives.CYLINDER';
info.MatPath{76} = 'start_state.attached_collision_objects.object.primitives.CONE';
info.MatPath{77} = 'start_state.attached_collision_objects.object.primitives.type';
info.MatPath{78} = 'start_state.attached_collision_objects.object.primitives.dimensions';
info.MatPath{79} = 'start_state.attached_collision_objects.object.primitives.BOX_X';
info.MatPath{80} = 'start_state.attached_collision_objects.object.primitives.BOX_Y';
info.MatPath{81} = 'start_state.attached_collision_objects.object.primitives.BOX_Z';
info.MatPath{82} = 'start_state.attached_collision_objects.object.primitives.SPHERE_RADIUS';
info.MatPath{83} = 'start_state.attached_collision_objects.object.primitives.CYLINDER_HEIGHT';
info.MatPath{84} = 'start_state.attached_collision_objects.object.primitives.CYLINDER_RADIUS';
info.MatPath{85} = 'start_state.attached_collision_objects.object.primitives.CONE_HEIGHT';
info.MatPath{86} = 'start_state.attached_collision_objects.object.primitives.CONE_RADIUS';
info.MatPath{87} = 'start_state.attached_collision_objects.object.primitive_poses';
info.MatPath{88} = 'start_state.attached_collision_objects.object.primitive_poses.position';
info.MatPath{89} = 'start_state.attached_collision_objects.object.primitive_poses.position.x';
info.MatPath{90} = 'start_state.attached_collision_objects.object.primitive_poses.position.y';
info.MatPath{91} = 'start_state.attached_collision_objects.object.primitive_poses.position.z';
info.MatPath{92} = 'start_state.attached_collision_objects.object.primitive_poses.orientation';
info.MatPath{93} = 'start_state.attached_collision_objects.object.primitive_poses.orientation.x';
info.MatPath{94} = 'start_state.attached_collision_objects.object.primitive_poses.orientation.y';
info.MatPath{95} = 'start_state.attached_collision_objects.object.primitive_poses.orientation.z';
info.MatPath{96} = 'start_state.attached_collision_objects.object.primitive_poses.orientation.w';
info.MatPath{97} = 'start_state.attached_collision_objects.object.meshes';
info.MatPath{98} = 'start_state.attached_collision_objects.object.meshes.triangles';
info.MatPath{99} = 'start_state.attached_collision_objects.object.meshes.triangles.vertex_indices';
info.MatPath{100} = 'start_state.attached_collision_objects.object.meshes.vertices';
info.MatPath{101} = 'start_state.attached_collision_objects.object.meshes.vertices.x';
info.MatPath{102} = 'start_state.attached_collision_objects.object.meshes.vertices.y';
info.MatPath{103} = 'start_state.attached_collision_objects.object.meshes.vertices.z';
info.MatPath{104} = 'start_state.attached_collision_objects.object.mesh_poses';
info.MatPath{105} = 'start_state.attached_collision_objects.object.mesh_poses.position';
info.MatPath{106} = 'start_state.attached_collision_objects.object.mesh_poses.position.x';
info.MatPath{107} = 'start_state.attached_collision_objects.object.mesh_poses.position.y';
info.MatPath{108} = 'start_state.attached_collision_objects.object.mesh_poses.position.z';
info.MatPath{109} = 'start_state.attached_collision_objects.object.mesh_poses.orientation';
info.MatPath{110} = 'start_state.attached_collision_objects.object.mesh_poses.orientation.x';
info.MatPath{111} = 'start_state.attached_collision_objects.object.mesh_poses.orientation.y';
info.MatPath{112} = 'start_state.attached_collision_objects.object.mesh_poses.orientation.z';
info.MatPath{113} = 'start_state.attached_collision_objects.object.mesh_poses.orientation.w';
info.MatPath{114} = 'start_state.attached_collision_objects.object.planes';
info.MatPath{115} = 'start_state.attached_collision_objects.object.planes.coef';
info.MatPath{116} = 'start_state.attached_collision_objects.object.plane_poses';
info.MatPath{117} = 'start_state.attached_collision_objects.object.plane_poses.position';
info.MatPath{118} = 'start_state.attached_collision_objects.object.plane_poses.position.x';
info.MatPath{119} = 'start_state.attached_collision_objects.object.plane_poses.position.y';
info.MatPath{120} = 'start_state.attached_collision_objects.object.plane_poses.position.z';
info.MatPath{121} = 'start_state.attached_collision_objects.object.plane_poses.orientation';
info.MatPath{122} = 'start_state.attached_collision_objects.object.plane_poses.orientation.x';
info.MatPath{123} = 'start_state.attached_collision_objects.object.plane_poses.orientation.y';
info.MatPath{124} = 'start_state.attached_collision_objects.object.plane_poses.orientation.z';
info.MatPath{125} = 'start_state.attached_collision_objects.object.plane_poses.orientation.w';
info.MatPath{126} = 'start_state.attached_collision_objects.object.subframe_names';
info.MatPath{127} = 'start_state.attached_collision_objects.object.subframe_poses';
info.MatPath{128} = 'start_state.attached_collision_objects.object.subframe_poses.position';
info.MatPath{129} = 'start_state.attached_collision_objects.object.subframe_poses.position.x';
info.MatPath{130} = 'start_state.attached_collision_objects.object.subframe_poses.position.y';
info.MatPath{131} = 'start_state.attached_collision_objects.object.subframe_poses.position.z';
info.MatPath{132} = 'start_state.attached_collision_objects.object.subframe_poses.orientation';
info.MatPath{133} = 'start_state.attached_collision_objects.object.subframe_poses.orientation.x';
info.MatPath{134} = 'start_state.attached_collision_objects.object.subframe_poses.orientation.y';
info.MatPath{135} = 'start_state.attached_collision_objects.object.subframe_poses.orientation.z';
info.MatPath{136} = 'start_state.attached_collision_objects.object.subframe_poses.orientation.w';
info.MatPath{137} = 'start_state.attached_collision_objects.object.ADD';
info.MatPath{138} = 'start_state.attached_collision_objects.object.REMOVE';
info.MatPath{139} = 'start_state.attached_collision_objects.object.APPEND';
info.MatPath{140} = 'start_state.attached_collision_objects.object.MOVE';
info.MatPath{141} = 'start_state.attached_collision_objects.object.operation';
info.MatPath{142} = 'start_state.attached_collision_objects.touch_links';
info.MatPath{143} = 'start_state.attached_collision_objects.detach_posture';
info.MatPath{144} = 'start_state.attached_collision_objects.detach_posture.header';
info.MatPath{145} = 'start_state.attached_collision_objects.detach_posture.header.seq';
info.MatPath{146} = 'start_state.attached_collision_objects.detach_posture.header.stamp';
info.MatPath{147} = 'start_state.attached_collision_objects.detach_posture.header.stamp.sec';
info.MatPath{148} = 'start_state.attached_collision_objects.detach_posture.header.stamp.nsec';
info.MatPath{149} = 'start_state.attached_collision_objects.detach_posture.header.frame_id';
info.MatPath{150} = 'start_state.attached_collision_objects.detach_posture.joint_names';
info.MatPath{151} = 'start_state.attached_collision_objects.detach_posture.points';
info.MatPath{152} = 'start_state.attached_collision_objects.detach_posture.points.positions';
info.MatPath{153} = 'start_state.attached_collision_objects.detach_posture.points.velocities';
info.MatPath{154} = 'start_state.attached_collision_objects.detach_posture.points.accelerations';
info.MatPath{155} = 'start_state.attached_collision_objects.detach_posture.points.effort';
info.MatPath{156} = 'start_state.attached_collision_objects.detach_posture.points.time_from_start';
info.MatPath{157} = 'start_state.attached_collision_objects.detach_posture.points.time_from_start.sec';
info.MatPath{158} = 'start_state.attached_collision_objects.detach_posture.points.time_from_start.nsec';
info.MatPath{159} = 'start_state.attached_collision_objects.weight';
info.MatPath{160} = 'start_state.is_diff';
info.MatPath{161} = 'solution';
info.MatPath{162} = 'solution.joint_trajectory';
info.MatPath{163} = 'solution.joint_trajectory.header';
info.MatPath{164} = 'solution.joint_trajectory.header.seq';
info.MatPath{165} = 'solution.joint_trajectory.header.stamp';
info.MatPath{166} = 'solution.joint_trajectory.header.stamp.sec';
info.MatPath{167} = 'solution.joint_trajectory.header.stamp.nsec';
info.MatPath{168} = 'solution.joint_trajectory.header.frame_id';
info.MatPath{169} = 'solution.joint_trajectory.joint_names';
info.MatPath{170} = 'solution.joint_trajectory.points';
info.MatPath{171} = 'solution.joint_trajectory.points.positions';
info.MatPath{172} = 'solution.joint_trajectory.points.velocities';
info.MatPath{173} = 'solution.joint_trajectory.points.accelerations';
info.MatPath{174} = 'solution.joint_trajectory.points.effort';
info.MatPath{175} = 'solution.joint_trajectory.points.time_from_start';
info.MatPath{176} = 'solution.joint_trajectory.points.time_from_start.sec';
info.MatPath{177} = 'solution.joint_trajectory.points.time_from_start.nsec';
info.MatPath{178} = 'solution.multi_dof_joint_trajectory';
info.MatPath{179} = 'solution.multi_dof_joint_trajectory.header';
info.MatPath{180} = 'solution.multi_dof_joint_trajectory.header.seq';
info.MatPath{181} = 'solution.multi_dof_joint_trajectory.header.stamp';
info.MatPath{182} = 'solution.multi_dof_joint_trajectory.header.stamp.sec';
info.MatPath{183} = 'solution.multi_dof_joint_trajectory.header.stamp.nsec';
info.MatPath{184} = 'solution.multi_dof_joint_trajectory.header.frame_id';
info.MatPath{185} = 'solution.multi_dof_joint_trajectory.joint_names';
info.MatPath{186} = 'solution.multi_dof_joint_trajectory.points';
info.MatPath{187} = 'solution.multi_dof_joint_trajectory.points.transforms';
info.MatPath{188} = 'solution.multi_dof_joint_trajectory.points.transforms.translation';
info.MatPath{189} = 'solution.multi_dof_joint_trajectory.points.transforms.translation.x';
info.MatPath{190} = 'solution.multi_dof_joint_trajectory.points.transforms.translation.y';
info.MatPath{191} = 'solution.multi_dof_joint_trajectory.points.transforms.translation.z';
info.MatPath{192} = 'solution.multi_dof_joint_trajectory.points.transforms.rotation';
info.MatPath{193} = 'solution.multi_dof_joint_trajectory.points.transforms.rotation.x';
info.MatPath{194} = 'solution.multi_dof_joint_trajectory.points.transforms.rotation.y';
info.MatPath{195} = 'solution.multi_dof_joint_trajectory.points.transforms.rotation.z';
info.MatPath{196} = 'solution.multi_dof_joint_trajectory.points.transforms.rotation.w';
info.MatPath{197} = 'solution.multi_dof_joint_trajectory.points.velocities';
info.MatPath{198} = 'solution.multi_dof_joint_trajectory.points.velocities.linear';
info.MatPath{199} = 'solution.multi_dof_joint_trajectory.points.velocities.linear.x';
info.MatPath{200} = 'solution.multi_dof_joint_trajectory.points.velocities.linear.y';
info.MatPath{201} = 'solution.multi_dof_joint_trajectory.points.velocities.linear.z';
info.MatPath{202} = 'solution.multi_dof_joint_trajectory.points.velocities.angular';
info.MatPath{203} = 'solution.multi_dof_joint_trajectory.points.velocities.angular.x';
info.MatPath{204} = 'solution.multi_dof_joint_trajectory.points.velocities.angular.y';
info.MatPath{205} = 'solution.multi_dof_joint_trajectory.points.velocities.angular.z';
info.MatPath{206} = 'solution.multi_dof_joint_trajectory.points.accelerations';
info.MatPath{207} = 'solution.multi_dof_joint_trajectory.points.accelerations.linear';
info.MatPath{208} = 'solution.multi_dof_joint_trajectory.points.accelerations.linear.x';
info.MatPath{209} = 'solution.multi_dof_joint_trajectory.points.accelerations.linear.y';
info.MatPath{210} = 'solution.multi_dof_joint_trajectory.points.accelerations.linear.z';
info.MatPath{211} = 'solution.multi_dof_joint_trajectory.points.accelerations.angular';
info.MatPath{212} = 'solution.multi_dof_joint_trajectory.points.accelerations.angular.x';
info.MatPath{213} = 'solution.multi_dof_joint_trajectory.points.accelerations.angular.y';
info.MatPath{214} = 'solution.multi_dof_joint_trajectory.points.accelerations.angular.z';
info.MatPath{215} = 'solution.multi_dof_joint_trajectory.points.time_from_start';
info.MatPath{216} = 'solution.multi_dof_joint_trajectory.points.time_from_start.sec';
info.MatPath{217} = 'solution.multi_dof_joint_trajectory.points.time_from_start.nsec';
info.MatPath{218} = 'fraction';
info.MatPath{219} = 'error_code';
info.MatPath{220} = 'error_code.val';
info.MatPath{221} = 'error_code.SUCCESS';
info.MatPath{222} = 'error_code.FAILURE';
info.MatPath{223} = 'error_code.PLANNING_FAILED';
info.MatPath{224} = 'error_code.INVALID_MOTION_PLAN';
info.MatPath{225} = 'error_code.MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE';
info.MatPath{226} = 'error_code.CONTROL_FAILED';
info.MatPath{227} = 'error_code.UNABLE_TO_AQUIRE_SENSOR_DATA';
info.MatPath{228} = 'error_code.TIMED_OUT';
info.MatPath{229} = 'error_code.PREEMPTED';
info.MatPath{230} = 'error_code.START_STATE_IN_COLLISION';
info.MatPath{231} = 'error_code.START_STATE_VIOLATES_PATH_CONSTRAINTS';
info.MatPath{232} = 'error_code.GOAL_IN_COLLISION';
info.MatPath{233} = 'error_code.GOAL_VIOLATES_PATH_CONSTRAINTS';
info.MatPath{234} = 'error_code.GOAL_CONSTRAINTS_VIOLATED';
info.MatPath{235} = 'error_code.INVALID_GROUP_NAME';
info.MatPath{236} = 'error_code.INVALID_GOAL_CONSTRAINTS';
info.MatPath{237} = 'error_code.INVALID_ROBOT_STATE';
info.MatPath{238} = 'error_code.INVALID_LINK_NAME';
info.MatPath{239} = 'error_code.INVALID_OBJECT_NAME';
info.MatPath{240} = 'error_code.FRAME_TRANSFORM_FAILURE';
info.MatPath{241} = 'error_code.COLLISION_CHECKING_UNAVAILABLE';
info.MatPath{242} = 'error_code.ROBOT_STATE_STALE';
info.MatPath{243} = 'error_code.SENSOR_INFO_STALE';
info.MatPath{244} = 'error_code.COMMUNICATION_FAILURE';
info.MatPath{245} = 'error_code.NO_IK_SOLUTION';
