function [data, info] = moveGroupActionResult
%MoveGroupActionResult gives an empty data for moveit_msgs/MoveGroupActionResult

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'moveit_msgs/MoveGroupActionResult';
[data.Header, info.Header] = ros.internal.ros.messages.std_msgs.header;
info.Header.MLdataType = 'struct';
[data.Status, info.Status] = ros.internal.ros.messages.actionlib_msgs.goalStatus;
info.Status.MLdataType = 'struct';
[data.Result, info.Result] = ros.internal.ros.messages.moveit_msgs.moveGroupResult;
info.Result.MLdataType = 'struct';
info.MessageType = 'moveit_msgs/MoveGroupActionResult';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,327);
info.MatPath{1} = 'header';
info.MatPath{2} = 'header.seq';
info.MatPath{3} = 'header.stamp';
info.MatPath{4} = 'header.stamp.sec';
info.MatPath{5} = 'header.stamp.nsec';
info.MatPath{6} = 'header.frame_id';
info.MatPath{7} = 'status';
info.MatPath{8} = 'status.goal_id';
info.MatPath{9} = 'status.goal_id.stamp';
info.MatPath{10} = 'status.goal_id.stamp.sec';
info.MatPath{11} = 'status.goal_id.stamp.nsec';
info.MatPath{12} = 'status.goal_id.id';
info.MatPath{13} = 'status.status';
info.MatPath{14} = 'status.PENDING';
info.MatPath{15} = 'status.ACTIVE';
info.MatPath{16} = 'status.PREEMPTED';
info.MatPath{17} = 'status.SUCCEEDED';
info.MatPath{18} = 'status.ABORTED';
info.MatPath{19} = 'status.REJECTED';
info.MatPath{20} = 'status.PREEMPTING';
info.MatPath{21} = 'status.RECALLING';
info.MatPath{22} = 'status.RECALLED';
info.MatPath{23} = 'status.LOST';
info.MatPath{24} = 'status.text';
info.MatPath{25} = 'result';
info.MatPath{26} = 'result.error_code';
info.MatPath{27} = 'result.error_code.val';
info.MatPath{28} = 'result.error_code.SUCCESS';
info.MatPath{29} = 'result.error_code.FAILURE';
info.MatPath{30} = 'result.error_code.PLANNING_FAILED';
info.MatPath{31} = 'result.error_code.INVALID_MOTION_PLAN';
info.MatPath{32} = 'result.error_code.MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE';
info.MatPath{33} = 'result.error_code.CONTROL_FAILED';
info.MatPath{34} = 'result.error_code.UNABLE_TO_AQUIRE_SENSOR_DATA';
info.MatPath{35} = 'result.error_code.TIMED_OUT';
info.MatPath{36} = 'result.error_code.PREEMPTED';
info.MatPath{37} = 'result.error_code.START_STATE_IN_COLLISION';
info.MatPath{38} = 'result.error_code.START_STATE_VIOLATES_PATH_CONSTRAINTS';
info.MatPath{39} = 'result.error_code.GOAL_IN_COLLISION';
info.MatPath{40} = 'result.error_code.GOAL_VIOLATES_PATH_CONSTRAINTS';
info.MatPath{41} = 'result.error_code.GOAL_CONSTRAINTS_VIOLATED';
info.MatPath{42} = 'result.error_code.INVALID_GROUP_NAME';
info.MatPath{43} = 'result.error_code.INVALID_GOAL_CONSTRAINTS';
info.MatPath{44} = 'result.error_code.INVALID_ROBOT_STATE';
info.MatPath{45} = 'result.error_code.INVALID_LINK_NAME';
info.MatPath{46} = 'result.error_code.INVALID_OBJECT_NAME';
info.MatPath{47} = 'result.error_code.FRAME_TRANSFORM_FAILURE';
info.MatPath{48} = 'result.error_code.COLLISION_CHECKING_UNAVAILABLE';
info.MatPath{49} = 'result.error_code.ROBOT_STATE_STALE';
info.MatPath{50} = 'result.error_code.SENSOR_INFO_STALE';
info.MatPath{51} = 'result.error_code.COMMUNICATION_FAILURE';
info.MatPath{52} = 'result.error_code.NO_IK_SOLUTION';
info.MatPath{53} = 'result.trajectory_start';
info.MatPath{54} = 'result.trajectory_start.joint_state';
info.MatPath{55} = 'result.trajectory_start.joint_state.header';
info.MatPath{56} = 'result.trajectory_start.joint_state.header.seq';
info.MatPath{57} = 'result.trajectory_start.joint_state.header.stamp';
info.MatPath{58} = 'result.trajectory_start.joint_state.header.stamp.sec';
info.MatPath{59} = 'result.trajectory_start.joint_state.header.stamp.nsec';
info.MatPath{60} = 'result.trajectory_start.joint_state.header.frame_id';
info.MatPath{61} = 'result.trajectory_start.joint_state.name';
info.MatPath{62} = 'result.trajectory_start.joint_state.position';
info.MatPath{63} = 'result.trajectory_start.joint_state.velocity';
info.MatPath{64} = 'result.trajectory_start.joint_state.effort';
info.MatPath{65} = 'result.trajectory_start.multi_dof_joint_state';
info.MatPath{66} = 'result.trajectory_start.multi_dof_joint_state.header';
info.MatPath{67} = 'result.trajectory_start.multi_dof_joint_state.header.seq';
info.MatPath{68} = 'result.trajectory_start.multi_dof_joint_state.header.stamp';
info.MatPath{69} = 'result.trajectory_start.multi_dof_joint_state.header.stamp.sec';
info.MatPath{70} = 'result.trajectory_start.multi_dof_joint_state.header.stamp.nsec';
info.MatPath{71} = 'result.trajectory_start.multi_dof_joint_state.header.frame_id';
info.MatPath{72} = 'result.trajectory_start.multi_dof_joint_state.joint_names';
info.MatPath{73} = 'result.trajectory_start.multi_dof_joint_state.transforms';
info.MatPath{74} = 'result.trajectory_start.multi_dof_joint_state.transforms.translation';
info.MatPath{75} = 'result.trajectory_start.multi_dof_joint_state.transforms.translation.x';
info.MatPath{76} = 'result.trajectory_start.multi_dof_joint_state.transforms.translation.y';
info.MatPath{77} = 'result.trajectory_start.multi_dof_joint_state.transforms.translation.z';
info.MatPath{78} = 'result.trajectory_start.multi_dof_joint_state.transforms.rotation';
info.MatPath{79} = 'result.trajectory_start.multi_dof_joint_state.transforms.rotation.x';
info.MatPath{80} = 'result.trajectory_start.multi_dof_joint_state.transforms.rotation.y';
info.MatPath{81} = 'result.trajectory_start.multi_dof_joint_state.transforms.rotation.z';
info.MatPath{82} = 'result.trajectory_start.multi_dof_joint_state.transforms.rotation.w';
info.MatPath{83} = 'result.trajectory_start.multi_dof_joint_state.twist';
info.MatPath{84} = 'result.trajectory_start.multi_dof_joint_state.twist.linear';
info.MatPath{85} = 'result.trajectory_start.multi_dof_joint_state.twist.linear.x';
info.MatPath{86} = 'result.trajectory_start.multi_dof_joint_state.twist.linear.y';
info.MatPath{87} = 'result.trajectory_start.multi_dof_joint_state.twist.linear.z';
info.MatPath{88} = 'result.trajectory_start.multi_dof_joint_state.twist.angular';
info.MatPath{89} = 'result.trajectory_start.multi_dof_joint_state.twist.angular.x';
info.MatPath{90} = 'result.trajectory_start.multi_dof_joint_state.twist.angular.y';
info.MatPath{91} = 'result.trajectory_start.multi_dof_joint_state.twist.angular.z';
info.MatPath{92} = 'result.trajectory_start.multi_dof_joint_state.wrench';
info.MatPath{93} = 'result.trajectory_start.multi_dof_joint_state.wrench.force';
info.MatPath{94} = 'result.trajectory_start.multi_dof_joint_state.wrench.force.x';
info.MatPath{95} = 'result.trajectory_start.multi_dof_joint_state.wrench.force.y';
info.MatPath{96} = 'result.trajectory_start.multi_dof_joint_state.wrench.force.z';
info.MatPath{97} = 'result.trajectory_start.multi_dof_joint_state.wrench.torque';
info.MatPath{98} = 'result.trajectory_start.multi_dof_joint_state.wrench.torque.x';
info.MatPath{99} = 'result.trajectory_start.multi_dof_joint_state.wrench.torque.y';
info.MatPath{100} = 'result.trajectory_start.multi_dof_joint_state.wrench.torque.z';
info.MatPath{101} = 'result.trajectory_start.attached_collision_objects';
info.MatPath{102} = 'result.trajectory_start.attached_collision_objects.link_name';
info.MatPath{103} = 'result.trajectory_start.attached_collision_objects.object';
info.MatPath{104} = 'result.trajectory_start.attached_collision_objects.object.header';
info.MatPath{105} = 'result.trajectory_start.attached_collision_objects.object.header.seq';
info.MatPath{106} = 'result.trajectory_start.attached_collision_objects.object.header.stamp';
info.MatPath{107} = 'result.trajectory_start.attached_collision_objects.object.header.stamp.sec';
info.MatPath{108} = 'result.trajectory_start.attached_collision_objects.object.header.stamp.nsec';
info.MatPath{109} = 'result.trajectory_start.attached_collision_objects.object.header.frame_id';
info.MatPath{110} = 'result.trajectory_start.attached_collision_objects.object.pose';
info.MatPath{111} = 'result.trajectory_start.attached_collision_objects.object.pose.position';
info.MatPath{112} = 'result.trajectory_start.attached_collision_objects.object.pose.position.x';
info.MatPath{113} = 'result.trajectory_start.attached_collision_objects.object.pose.position.y';
info.MatPath{114} = 'result.trajectory_start.attached_collision_objects.object.pose.position.z';
info.MatPath{115} = 'result.trajectory_start.attached_collision_objects.object.pose.orientation';
info.MatPath{116} = 'result.trajectory_start.attached_collision_objects.object.pose.orientation.x';
info.MatPath{117} = 'result.trajectory_start.attached_collision_objects.object.pose.orientation.y';
info.MatPath{118} = 'result.trajectory_start.attached_collision_objects.object.pose.orientation.z';
info.MatPath{119} = 'result.trajectory_start.attached_collision_objects.object.pose.orientation.w';
info.MatPath{120} = 'result.trajectory_start.attached_collision_objects.object.id';
info.MatPath{121} = 'result.trajectory_start.attached_collision_objects.object.type';
info.MatPath{122} = 'result.trajectory_start.attached_collision_objects.object.type.key';
info.MatPath{123} = 'result.trajectory_start.attached_collision_objects.object.type.db';
info.MatPath{124} = 'result.trajectory_start.attached_collision_objects.object.primitives';
info.MatPath{125} = 'result.trajectory_start.attached_collision_objects.object.primitives.BOX';
info.MatPath{126} = 'result.trajectory_start.attached_collision_objects.object.primitives.SPHERE';
info.MatPath{127} = 'result.trajectory_start.attached_collision_objects.object.primitives.CYLINDER';
info.MatPath{128} = 'result.trajectory_start.attached_collision_objects.object.primitives.CONE';
info.MatPath{129} = 'result.trajectory_start.attached_collision_objects.object.primitives.type';
info.MatPath{130} = 'result.trajectory_start.attached_collision_objects.object.primitives.dimensions';
info.MatPath{131} = 'result.trajectory_start.attached_collision_objects.object.primitives.BOX_X';
info.MatPath{132} = 'result.trajectory_start.attached_collision_objects.object.primitives.BOX_Y';
info.MatPath{133} = 'result.trajectory_start.attached_collision_objects.object.primitives.BOX_Z';
info.MatPath{134} = 'result.trajectory_start.attached_collision_objects.object.primitives.SPHERE_RADIUS';
info.MatPath{135} = 'result.trajectory_start.attached_collision_objects.object.primitives.CYLINDER_HEIGHT';
info.MatPath{136} = 'result.trajectory_start.attached_collision_objects.object.primitives.CYLINDER_RADIUS';
info.MatPath{137} = 'result.trajectory_start.attached_collision_objects.object.primitives.CONE_HEIGHT';
info.MatPath{138} = 'result.trajectory_start.attached_collision_objects.object.primitives.CONE_RADIUS';
info.MatPath{139} = 'result.trajectory_start.attached_collision_objects.object.primitive_poses';
info.MatPath{140} = 'result.trajectory_start.attached_collision_objects.object.primitive_poses.position';
info.MatPath{141} = 'result.trajectory_start.attached_collision_objects.object.primitive_poses.position.x';
info.MatPath{142} = 'result.trajectory_start.attached_collision_objects.object.primitive_poses.position.y';
info.MatPath{143} = 'result.trajectory_start.attached_collision_objects.object.primitive_poses.position.z';
info.MatPath{144} = 'result.trajectory_start.attached_collision_objects.object.primitive_poses.orientation';
info.MatPath{145} = 'result.trajectory_start.attached_collision_objects.object.primitive_poses.orientation.x';
info.MatPath{146} = 'result.trajectory_start.attached_collision_objects.object.primitive_poses.orientation.y';
info.MatPath{147} = 'result.trajectory_start.attached_collision_objects.object.primitive_poses.orientation.z';
info.MatPath{148} = 'result.trajectory_start.attached_collision_objects.object.primitive_poses.orientation.w';
info.MatPath{149} = 'result.trajectory_start.attached_collision_objects.object.meshes';
info.MatPath{150} = 'result.trajectory_start.attached_collision_objects.object.meshes.triangles';
info.MatPath{151} = 'result.trajectory_start.attached_collision_objects.object.meshes.triangles.vertex_indices';
info.MatPath{152} = 'result.trajectory_start.attached_collision_objects.object.meshes.vertices';
info.MatPath{153} = 'result.trajectory_start.attached_collision_objects.object.meshes.vertices.x';
info.MatPath{154} = 'result.trajectory_start.attached_collision_objects.object.meshes.vertices.y';
info.MatPath{155} = 'result.trajectory_start.attached_collision_objects.object.meshes.vertices.z';
info.MatPath{156} = 'result.trajectory_start.attached_collision_objects.object.mesh_poses';
info.MatPath{157} = 'result.trajectory_start.attached_collision_objects.object.mesh_poses.position';
info.MatPath{158} = 'result.trajectory_start.attached_collision_objects.object.mesh_poses.position.x';
info.MatPath{159} = 'result.trajectory_start.attached_collision_objects.object.mesh_poses.position.y';
info.MatPath{160} = 'result.trajectory_start.attached_collision_objects.object.mesh_poses.position.z';
info.MatPath{161} = 'result.trajectory_start.attached_collision_objects.object.mesh_poses.orientation';
info.MatPath{162} = 'result.trajectory_start.attached_collision_objects.object.mesh_poses.orientation.x';
info.MatPath{163} = 'result.trajectory_start.attached_collision_objects.object.mesh_poses.orientation.y';
info.MatPath{164} = 'result.trajectory_start.attached_collision_objects.object.mesh_poses.orientation.z';
info.MatPath{165} = 'result.trajectory_start.attached_collision_objects.object.mesh_poses.orientation.w';
info.MatPath{166} = 'result.trajectory_start.attached_collision_objects.object.planes';
info.MatPath{167} = 'result.trajectory_start.attached_collision_objects.object.planes.coef';
info.MatPath{168} = 'result.trajectory_start.attached_collision_objects.object.plane_poses';
info.MatPath{169} = 'result.trajectory_start.attached_collision_objects.object.plane_poses.position';
info.MatPath{170} = 'result.trajectory_start.attached_collision_objects.object.plane_poses.position.x';
info.MatPath{171} = 'result.trajectory_start.attached_collision_objects.object.plane_poses.position.y';
info.MatPath{172} = 'result.trajectory_start.attached_collision_objects.object.plane_poses.position.z';
info.MatPath{173} = 'result.trajectory_start.attached_collision_objects.object.plane_poses.orientation';
info.MatPath{174} = 'result.trajectory_start.attached_collision_objects.object.plane_poses.orientation.x';
info.MatPath{175} = 'result.trajectory_start.attached_collision_objects.object.plane_poses.orientation.y';
info.MatPath{176} = 'result.trajectory_start.attached_collision_objects.object.plane_poses.orientation.z';
info.MatPath{177} = 'result.trajectory_start.attached_collision_objects.object.plane_poses.orientation.w';
info.MatPath{178} = 'result.trajectory_start.attached_collision_objects.object.subframe_names';
info.MatPath{179} = 'result.trajectory_start.attached_collision_objects.object.subframe_poses';
info.MatPath{180} = 'result.trajectory_start.attached_collision_objects.object.subframe_poses.position';
info.MatPath{181} = 'result.trajectory_start.attached_collision_objects.object.subframe_poses.position.x';
info.MatPath{182} = 'result.trajectory_start.attached_collision_objects.object.subframe_poses.position.y';
info.MatPath{183} = 'result.trajectory_start.attached_collision_objects.object.subframe_poses.position.z';
info.MatPath{184} = 'result.trajectory_start.attached_collision_objects.object.subframe_poses.orientation';
info.MatPath{185} = 'result.trajectory_start.attached_collision_objects.object.subframe_poses.orientation.x';
info.MatPath{186} = 'result.trajectory_start.attached_collision_objects.object.subframe_poses.orientation.y';
info.MatPath{187} = 'result.trajectory_start.attached_collision_objects.object.subframe_poses.orientation.z';
info.MatPath{188} = 'result.trajectory_start.attached_collision_objects.object.subframe_poses.orientation.w';
info.MatPath{189} = 'result.trajectory_start.attached_collision_objects.object.ADD';
info.MatPath{190} = 'result.trajectory_start.attached_collision_objects.object.REMOVE';
info.MatPath{191} = 'result.trajectory_start.attached_collision_objects.object.APPEND';
info.MatPath{192} = 'result.trajectory_start.attached_collision_objects.object.MOVE';
info.MatPath{193} = 'result.trajectory_start.attached_collision_objects.object.operation';
info.MatPath{194} = 'result.trajectory_start.attached_collision_objects.touch_links';
info.MatPath{195} = 'result.trajectory_start.attached_collision_objects.detach_posture';
info.MatPath{196} = 'result.trajectory_start.attached_collision_objects.detach_posture.header';
info.MatPath{197} = 'result.trajectory_start.attached_collision_objects.detach_posture.header.seq';
info.MatPath{198} = 'result.trajectory_start.attached_collision_objects.detach_posture.header.stamp';
info.MatPath{199} = 'result.trajectory_start.attached_collision_objects.detach_posture.header.stamp.sec';
info.MatPath{200} = 'result.trajectory_start.attached_collision_objects.detach_posture.header.stamp.nsec';
info.MatPath{201} = 'result.trajectory_start.attached_collision_objects.detach_posture.header.frame_id';
info.MatPath{202} = 'result.trajectory_start.attached_collision_objects.detach_posture.joint_names';
info.MatPath{203} = 'result.trajectory_start.attached_collision_objects.detach_posture.points';
info.MatPath{204} = 'result.trajectory_start.attached_collision_objects.detach_posture.points.positions';
info.MatPath{205} = 'result.trajectory_start.attached_collision_objects.detach_posture.points.velocities';
info.MatPath{206} = 'result.trajectory_start.attached_collision_objects.detach_posture.points.accelerations';
info.MatPath{207} = 'result.trajectory_start.attached_collision_objects.detach_posture.points.effort';
info.MatPath{208} = 'result.trajectory_start.attached_collision_objects.detach_posture.points.time_from_start';
info.MatPath{209} = 'result.trajectory_start.attached_collision_objects.detach_posture.points.time_from_start.sec';
info.MatPath{210} = 'result.trajectory_start.attached_collision_objects.detach_posture.points.time_from_start.nsec';
info.MatPath{211} = 'result.trajectory_start.attached_collision_objects.weight';
info.MatPath{212} = 'result.trajectory_start.is_diff';
info.MatPath{213} = 'result.planned_trajectory';
info.MatPath{214} = 'result.planned_trajectory.joint_trajectory';
info.MatPath{215} = 'result.planned_trajectory.joint_trajectory.header';
info.MatPath{216} = 'result.planned_trajectory.joint_trajectory.header.seq';
info.MatPath{217} = 'result.planned_trajectory.joint_trajectory.header.stamp';
info.MatPath{218} = 'result.planned_trajectory.joint_trajectory.header.stamp.sec';
info.MatPath{219} = 'result.planned_trajectory.joint_trajectory.header.stamp.nsec';
info.MatPath{220} = 'result.planned_trajectory.joint_trajectory.header.frame_id';
info.MatPath{221} = 'result.planned_trajectory.joint_trajectory.joint_names';
info.MatPath{222} = 'result.planned_trajectory.joint_trajectory.points';
info.MatPath{223} = 'result.planned_trajectory.joint_trajectory.points.positions';
info.MatPath{224} = 'result.planned_trajectory.joint_trajectory.points.velocities';
info.MatPath{225} = 'result.planned_trajectory.joint_trajectory.points.accelerations';
info.MatPath{226} = 'result.planned_trajectory.joint_trajectory.points.effort';
info.MatPath{227} = 'result.planned_trajectory.joint_trajectory.points.time_from_start';
info.MatPath{228} = 'result.planned_trajectory.joint_trajectory.points.time_from_start.sec';
info.MatPath{229} = 'result.planned_trajectory.joint_trajectory.points.time_from_start.nsec';
info.MatPath{230} = 'result.planned_trajectory.multi_dof_joint_trajectory';
info.MatPath{231} = 'result.planned_trajectory.multi_dof_joint_trajectory.header';
info.MatPath{232} = 'result.planned_trajectory.multi_dof_joint_trajectory.header.seq';
info.MatPath{233} = 'result.planned_trajectory.multi_dof_joint_trajectory.header.stamp';
info.MatPath{234} = 'result.planned_trajectory.multi_dof_joint_trajectory.header.stamp.sec';
info.MatPath{235} = 'result.planned_trajectory.multi_dof_joint_trajectory.header.stamp.nsec';
info.MatPath{236} = 'result.planned_trajectory.multi_dof_joint_trajectory.header.frame_id';
info.MatPath{237} = 'result.planned_trajectory.multi_dof_joint_trajectory.joint_names';
info.MatPath{238} = 'result.planned_trajectory.multi_dof_joint_trajectory.points';
info.MatPath{239} = 'result.planned_trajectory.multi_dof_joint_trajectory.points.transforms';
info.MatPath{240} = 'result.planned_trajectory.multi_dof_joint_trajectory.points.transforms.translation';
info.MatPath{241} = 'result.planned_trajectory.multi_dof_joint_trajectory.points.transforms.translation.x';
info.MatPath{242} = 'result.planned_trajectory.multi_dof_joint_trajectory.points.transforms.translation.y';
info.MatPath{243} = 'result.planned_trajectory.multi_dof_joint_trajectory.points.transforms.translation.z';
info.MatPath{244} = 'result.planned_trajectory.multi_dof_joint_trajectory.points.transforms.rotation';
info.MatPath{245} = 'result.planned_trajectory.multi_dof_joint_trajectory.points.transforms.rotation.x';
info.MatPath{246} = 'result.planned_trajectory.multi_dof_joint_trajectory.points.transforms.rotation.y';
info.MatPath{247} = 'result.planned_trajectory.multi_dof_joint_trajectory.points.transforms.rotation.z';
info.MatPath{248} = 'result.planned_trajectory.multi_dof_joint_trajectory.points.transforms.rotation.w';
info.MatPath{249} = 'result.planned_trajectory.multi_dof_joint_trajectory.points.velocities';
info.MatPath{250} = 'result.planned_trajectory.multi_dof_joint_trajectory.points.velocities.linear';
info.MatPath{251} = 'result.planned_trajectory.multi_dof_joint_trajectory.points.velocities.linear.x';
info.MatPath{252} = 'result.planned_trajectory.multi_dof_joint_trajectory.points.velocities.linear.y';
info.MatPath{253} = 'result.planned_trajectory.multi_dof_joint_trajectory.points.velocities.linear.z';
info.MatPath{254} = 'result.planned_trajectory.multi_dof_joint_trajectory.points.velocities.angular';
info.MatPath{255} = 'result.planned_trajectory.multi_dof_joint_trajectory.points.velocities.angular.x';
info.MatPath{256} = 'result.planned_trajectory.multi_dof_joint_trajectory.points.velocities.angular.y';
info.MatPath{257} = 'result.planned_trajectory.multi_dof_joint_trajectory.points.velocities.angular.z';
info.MatPath{258} = 'result.planned_trajectory.multi_dof_joint_trajectory.points.accelerations';
info.MatPath{259} = 'result.planned_trajectory.multi_dof_joint_trajectory.points.accelerations.linear';
info.MatPath{260} = 'result.planned_trajectory.multi_dof_joint_trajectory.points.accelerations.linear.x';
info.MatPath{261} = 'result.planned_trajectory.multi_dof_joint_trajectory.points.accelerations.linear.y';
info.MatPath{262} = 'result.planned_trajectory.multi_dof_joint_trajectory.points.accelerations.linear.z';
info.MatPath{263} = 'result.planned_trajectory.multi_dof_joint_trajectory.points.accelerations.angular';
info.MatPath{264} = 'result.planned_trajectory.multi_dof_joint_trajectory.points.accelerations.angular.x';
info.MatPath{265} = 'result.planned_trajectory.multi_dof_joint_trajectory.points.accelerations.angular.y';
info.MatPath{266} = 'result.planned_trajectory.multi_dof_joint_trajectory.points.accelerations.angular.z';
info.MatPath{267} = 'result.planned_trajectory.multi_dof_joint_trajectory.points.time_from_start';
info.MatPath{268} = 'result.planned_trajectory.multi_dof_joint_trajectory.points.time_from_start.sec';
info.MatPath{269} = 'result.planned_trajectory.multi_dof_joint_trajectory.points.time_from_start.nsec';
info.MatPath{270} = 'result.executed_trajectory';
info.MatPath{271} = 'result.executed_trajectory.joint_trajectory';
info.MatPath{272} = 'result.executed_trajectory.joint_trajectory.header';
info.MatPath{273} = 'result.executed_trajectory.joint_trajectory.header.seq';
info.MatPath{274} = 'result.executed_trajectory.joint_trajectory.header.stamp';
info.MatPath{275} = 'result.executed_trajectory.joint_trajectory.header.stamp.sec';
info.MatPath{276} = 'result.executed_trajectory.joint_trajectory.header.stamp.nsec';
info.MatPath{277} = 'result.executed_trajectory.joint_trajectory.header.frame_id';
info.MatPath{278} = 'result.executed_trajectory.joint_trajectory.joint_names';
info.MatPath{279} = 'result.executed_trajectory.joint_trajectory.points';
info.MatPath{280} = 'result.executed_trajectory.joint_trajectory.points.positions';
info.MatPath{281} = 'result.executed_trajectory.joint_trajectory.points.velocities';
info.MatPath{282} = 'result.executed_trajectory.joint_trajectory.points.accelerations';
info.MatPath{283} = 'result.executed_trajectory.joint_trajectory.points.effort';
info.MatPath{284} = 'result.executed_trajectory.joint_trajectory.points.time_from_start';
info.MatPath{285} = 'result.executed_trajectory.joint_trajectory.points.time_from_start.sec';
info.MatPath{286} = 'result.executed_trajectory.joint_trajectory.points.time_from_start.nsec';
info.MatPath{287} = 'result.executed_trajectory.multi_dof_joint_trajectory';
info.MatPath{288} = 'result.executed_trajectory.multi_dof_joint_trajectory.header';
info.MatPath{289} = 'result.executed_trajectory.multi_dof_joint_trajectory.header.seq';
info.MatPath{290} = 'result.executed_trajectory.multi_dof_joint_trajectory.header.stamp';
info.MatPath{291} = 'result.executed_trajectory.multi_dof_joint_trajectory.header.stamp.sec';
info.MatPath{292} = 'result.executed_trajectory.multi_dof_joint_trajectory.header.stamp.nsec';
info.MatPath{293} = 'result.executed_trajectory.multi_dof_joint_trajectory.header.frame_id';
info.MatPath{294} = 'result.executed_trajectory.multi_dof_joint_trajectory.joint_names';
info.MatPath{295} = 'result.executed_trajectory.multi_dof_joint_trajectory.points';
info.MatPath{296} = 'result.executed_trajectory.multi_dof_joint_trajectory.points.transforms';
info.MatPath{297} = 'result.executed_trajectory.multi_dof_joint_trajectory.points.transforms.translation';
info.MatPath{298} = 'result.executed_trajectory.multi_dof_joint_trajectory.points.transforms.translation.x';
info.MatPath{299} = 'result.executed_trajectory.multi_dof_joint_trajectory.points.transforms.translation.y';
info.MatPath{300} = 'result.executed_trajectory.multi_dof_joint_trajectory.points.transforms.translation.z';
info.MatPath{301} = 'result.executed_trajectory.multi_dof_joint_trajectory.points.transforms.rotation';
info.MatPath{302} = 'result.executed_trajectory.multi_dof_joint_trajectory.points.transforms.rotation.x';
info.MatPath{303} = 'result.executed_trajectory.multi_dof_joint_trajectory.points.transforms.rotation.y';
info.MatPath{304} = 'result.executed_trajectory.multi_dof_joint_trajectory.points.transforms.rotation.z';
info.MatPath{305} = 'result.executed_trajectory.multi_dof_joint_trajectory.points.transforms.rotation.w';
info.MatPath{306} = 'result.executed_trajectory.multi_dof_joint_trajectory.points.velocities';
info.MatPath{307} = 'result.executed_trajectory.multi_dof_joint_trajectory.points.velocities.linear';
info.MatPath{308} = 'result.executed_trajectory.multi_dof_joint_trajectory.points.velocities.linear.x';
info.MatPath{309} = 'result.executed_trajectory.multi_dof_joint_trajectory.points.velocities.linear.y';
info.MatPath{310} = 'result.executed_trajectory.multi_dof_joint_trajectory.points.velocities.linear.z';
info.MatPath{311} = 'result.executed_trajectory.multi_dof_joint_trajectory.points.velocities.angular';
info.MatPath{312} = 'result.executed_trajectory.multi_dof_joint_trajectory.points.velocities.angular.x';
info.MatPath{313} = 'result.executed_trajectory.multi_dof_joint_trajectory.points.velocities.angular.y';
info.MatPath{314} = 'result.executed_trajectory.multi_dof_joint_trajectory.points.velocities.angular.z';
info.MatPath{315} = 'result.executed_trajectory.multi_dof_joint_trajectory.points.accelerations';
info.MatPath{316} = 'result.executed_trajectory.multi_dof_joint_trajectory.points.accelerations.linear';
info.MatPath{317} = 'result.executed_trajectory.multi_dof_joint_trajectory.points.accelerations.linear.x';
info.MatPath{318} = 'result.executed_trajectory.multi_dof_joint_trajectory.points.accelerations.linear.y';
info.MatPath{319} = 'result.executed_trajectory.multi_dof_joint_trajectory.points.accelerations.linear.z';
info.MatPath{320} = 'result.executed_trajectory.multi_dof_joint_trajectory.points.accelerations.angular';
info.MatPath{321} = 'result.executed_trajectory.multi_dof_joint_trajectory.points.accelerations.angular.x';
info.MatPath{322} = 'result.executed_trajectory.multi_dof_joint_trajectory.points.accelerations.angular.y';
info.MatPath{323} = 'result.executed_trajectory.multi_dof_joint_trajectory.points.accelerations.angular.z';
info.MatPath{324} = 'result.executed_trajectory.multi_dof_joint_trajectory.points.time_from_start';
info.MatPath{325} = 'result.executed_trajectory.multi_dof_joint_trajectory.points.time_from_start.sec';
info.MatPath{326} = 'result.executed_trajectory.multi_dof_joint_trajectory.points.time_from_start.nsec';
info.MatPath{327} = 'result.planning_time';
