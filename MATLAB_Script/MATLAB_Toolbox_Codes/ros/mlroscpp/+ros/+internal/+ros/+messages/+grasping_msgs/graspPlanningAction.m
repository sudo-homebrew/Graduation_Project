function [data, info] = graspPlanningAction
%GraspPlanningAction gives an empty data for grasping_msgs/GraspPlanningAction

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'grasping_msgs/GraspPlanningAction';
[data.ActionGoal, info.ActionGoal] = ros.internal.ros.messages.grasping_msgs.graspPlanningActionGoal;
info.ActionGoal.MLdataType = 'struct';
[data.ActionResult, info.ActionResult] = ros.internal.ros.messages.grasping_msgs.graspPlanningActionResult;
info.ActionResult.MLdataType = 'struct';
[data.ActionFeedback, info.ActionFeedback] = ros.internal.ros.messages.grasping_msgs.graspPlanningActionFeedback;
info.ActionFeedback.MLdataType = 'struct';
info.MessageType = 'grasping_msgs/GraspPlanningAction';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,341);
info.MatPath{1} = 'action_goal';
info.MatPath{2} = 'action_goal.header';
info.MatPath{3} = 'action_goal.header.seq';
info.MatPath{4} = 'action_goal.header.stamp';
info.MatPath{5} = 'action_goal.header.stamp.sec';
info.MatPath{6} = 'action_goal.header.stamp.nsec';
info.MatPath{7} = 'action_goal.header.frame_id';
info.MatPath{8} = 'action_goal.goal_id';
info.MatPath{9} = 'action_goal.goal_id.stamp';
info.MatPath{10} = 'action_goal.goal_id.stamp.sec';
info.MatPath{11} = 'action_goal.goal_id.stamp.nsec';
info.MatPath{12} = 'action_goal.goal_id.id';
info.MatPath{13} = 'action_goal.goal';
info.MatPath{14} = 'action_goal.goal.object';
info.MatPath{15} = 'action_goal.goal.object.header';
info.MatPath{16} = 'action_goal.goal.object.header.seq';
info.MatPath{17} = 'action_goal.goal.object.header.stamp';
info.MatPath{18} = 'action_goal.goal.object.header.stamp.sec';
info.MatPath{19} = 'action_goal.goal.object.header.stamp.nsec';
info.MatPath{20} = 'action_goal.goal.object.header.frame_id';
info.MatPath{21} = 'action_goal.goal.object.name';
info.MatPath{22} = 'action_goal.goal.object.support_surface';
info.MatPath{23} = 'action_goal.goal.object.properties';
info.MatPath{24} = 'action_goal.goal.object.properties.name';
info.MatPath{25} = 'action_goal.goal.object.properties.value';
info.MatPath{26} = 'action_goal.goal.object.point_cluster';
info.MatPath{27} = 'action_goal.goal.object.point_cluster.header';
info.MatPath{28} = 'action_goal.goal.object.point_cluster.header.seq';
info.MatPath{29} = 'action_goal.goal.object.point_cluster.header.stamp';
info.MatPath{30} = 'action_goal.goal.object.point_cluster.header.stamp.sec';
info.MatPath{31} = 'action_goal.goal.object.point_cluster.header.stamp.nsec';
info.MatPath{32} = 'action_goal.goal.object.point_cluster.header.frame_id';
info.MatPath{33} = 'action_goal.goal.object.point_cluster.height';
info.MatPath{34} = 'action_goal.goal.object.point_cluster.width';
info.MatPath{35} = 'action_goal.goal.object.point_cluster.fields';
info.MatPath{36} = 'action_goal.goal.object.point_cluster.fields.INT8';
info.MatPath{37} = 'action_goal.goal.object.point_cluster.fields.UINT8';
info.MatPath{38} = 'action_goal.goal.object.point_cluster.fields.INT16';
info.MatPath{39} = 'action_goal.goal.object.point_cluster.fields.UINT16';
info.MatPath{40} = 'action_goal.goal.object.point_cluster.fields.INT32';
info.MatPath{41} = 'action_goal.goal.object.point_cluster.fields.UINT32';
info.MatPath{42} = 'action_goal.goal.object.point_cluster.fields.FLOAT32';
info.MatPath{43} = 'action_goal.goal.object.point_cluster.fields.FLOAT64';
info.MatPath{44} = 'action_goal.goal.object.point_cluster.fields.name';
info.MatPath{45} = 'action_goal.goal.object.point_cluster.fields.offset';
info.MatPath{46} = 'action_goal.goal.object.point_cluster.fields.datatype';
info.MatPath{47} = 'action_goal.goal.object.point_cluster.fields.count';
info.MatPath{48} = 'action_goal.goal.object.point_cluster.is_bigendian';
info.MatPath{49} = 'action_goal.goal.object.point_cluster.point_step';
info.MatPath{50} = 'action_goal.goal.object.point_cluster.row_step';
info.MatPath{51} = 'action_goal.goal.object.point_cluster.data';
info.MatPath{52} = 'action_goal.goal.object.point_cluster.is_dense';
info.MatPath{53} = 'action_goal.goal.object.primitives';
info.MatPath{54} = 'action_goal.goal.object.primitives.BOX';
info.MatPath{55} = 'action_goal.goal.object.primitives.SPHERE';
info.MatPath{56} = 'action_goal.goal.object.primitives.CYLINDER';
info.MatPath{57} = 'action_goal.goal.object.primitives.CONE';
info.MatPath{58} = 'action_goal.goal.object.primitives.type';
info.MatPath{59} = 'action_goal.goal.object.primitives.dimensions';
info.MatPath{60} = 'action_goal.goal.object.primitives.BOX_X';
info.MatPath{61} = 'action_goal.goal.object.primitives.BOX_Y';
info.MatPath{62} = 'action_goal.goal.object.primitives.BOX_Z';
info.MatPath{63} = 'action_goal.goal.object.primitives.SPHERE_RADIUS';
info.MatPath{64} = 'action_goal.goal.object.primitives.CYLINDER_HEIGHT';
info.MatPath{65} = 'action_goal.goal.object.primitives.CYLINDER_RADIUS';
info.MatPath{66} = 'action_goal.goal.object.primitives.CONE_HEIGHT';
info.MatPath{67} = 'action_goal.goal.object.primitives.CONE_RADIUS';
info.MatPath{68} = 'action_goal.goal.object.primitive_poses';
info.MatPath{69} = 'action_goal.goal.object.primitive_poses.position';
info.MatPath{70} = 'action_goal.goal.object.primitive_poses.position.x';
info.MatPath{71} = 'action_goal.goal.object.primitive_poses.position.y';
info.MatPath{72} = 'action_goal.goal.object.primitive_poses.position.z';
info.MatPath{73} = 'action_goal.goal.object.primitive_poses.orientation';
info.MatPath{74} = 'action_goal.goal.object.primitive_poses.orientation.x';
info.MatPath{75} = 'action_goal.goal.object.primitive_poses.orientation.y';
info.MatPath{76} = 'action_goal.goal.object.primitive_poses.orientation.z';
info.MatPath{77} = 'action_goal.goal.object.primitive_poses.orientation.w';
info.MatPath{78} = 'action_goal.goal.object.meshes';
info.MatPath{79} = 'action_goal.goal.object.meshes.triangles';
info.MatPath{80} = 'action_goal.goal.object.meshes.triangles.vertex_indices';
info.MatPath{81} = 'action_goal.goal.object.meshes.vertices';
info.MatPath{82} = 'action_goal.goal.object.meshes.vertices.x';
info.MatPath{83} = 'action_goal.goal.object.meshes.vertices.y';
info.MatPath{84} = 'action_goal.goal.object.meshes.vertices.z';
info.MatPath{85} = 'action_goal.goal.object.mesh_poses';
info.MatPath{86} = 'action_goal.goal.object.mesh_poses.position';
info.MatPath{87} = 'action_goal.goal.object.mesh_poses.position.x';
info.MatPath{88} = 'action_goal.goal.object.mesh_poses.position.y';
info.MatPath{89} = 'action_goal.goal.object.mesh_poses.position.z';
info.MatPath{90} = 'action_goal.goal.object.mesh_poses.orientation';
info.MatPath{91} = 'action_goal.goal.object.mesh_poses.orientation.x';
info.MatPath{92} = 'action_goal.goal.object.mesh_poses.orientation.y';
info.MatPath{93} = 'action_goal.goal.object.mesh_poses.orientation.z';
info.MatPath{94} = 'action_goal.goal.object.mesh_poses.orientation.w';
info.MatPath{95} = 'action_goal.goal.object.surface';
info.MatPath{96} = 'action_goal.goal.object.surface.coef';
info.MatPath{97} = 'action_goal.goal.group_name';
info.MatPath{98} = 'action_result';
info.MatPath{99} = 'action_result.header';
info.MatPath{100} = 'action_result.header.seq';
info.MatPath{101} = 'action_result.header.stamp';
info.MatPath{102} = 'action_result.header.stamp.sec';
info.MatPath{103} = 'action_result.header.stamp.nsec';
info.MatPath{104} = 'action_result.header.frame_id';
info.MatPath{105} = 'action_result.status';
info.MatPath{106} = 'action_result.status.goal_id';
info.MatPath{107} = 'action_result.status.goal_id.stamp';
info.MatPath{108} = 'action_result.status.goal_id.stamp.sec';
info.MatPath{109} = 'action_result.status.goal_id.stamp.nsec';
info.MatPath{110} = 'action_result.status.goal_id.id';
info.MatPath{111} = 'action_result.status.status';
info.MatPath{112} = 'action_result.status.PENDING';
info.MatPath{113} = 'action_result.status.ACTIVE';
info.MatPath{114} = 'action_result.status.PREEMPTED';
info.MatPath{115} = 'action_result.status.SUCCEEDED';
info.MatPath{116} = 'action_result.status.ABORTED';
info.MatPath{117} = 'action_result.status.REJECTED';
info.MatPath{118} = 'action_result.status.PREEMPTING';
info.MatPath{119} = 'action_result.status.RECALLING';
info.MatPath{120} = 'action_result.status.RECALLED';
info.MatPath{121} = 'action_result.status.LOST';
info.MatPath{122} = 'action_result.status.text';
info.MatPath{123} = 'action_result.result';
info.MatPath{124} = 'action_result.result.grasps';
info.MatPath{125} = 'action_result.result.grasps.id';
info.MatPath{126} = 'action_result.result.grasps.pre_grasp_posture';
info.MatPath{127} = 'action_result.result.grasps.pre_grasp_posture.header';
info.MatPath{128} = 'action_result.result.grasps.pre_grasp_posture.header.seq';
info.MatPath{129} = 'action_result.result.grasps.pre_grasp_posture.header.stamp';
info.MatPath{130} = 'action_result.result.grasps.pre_grasp_posture.header.stamp.sec';
info.MatPath{131} = 'action_result.result.grasps.pre_grasp_posture.header.stamp.nsec';
info.MatPath{132} = 'action_result.result.grasps.pre_grasp_posture.header.frame_id';
info.MatPath{133} = 'action_result.result.grasps.pre_grasp_posture.joint_names';
info.MatPath{134} = 'action_result.result.grasps.pre_grasp_posture.points';
info.MatPath{135} = 'action_result.result.grasps.pre_grasp_posture.points.positions';
info.MatPath{136} = 'action_result.result.grasps.pre_grasp_posture.points.velocities';
info.MatPath{137} = 'action_result.result.grasps.pre_grasp_posture.points.accelerations';
info.MatPath{138} = 'action_result.result.grasps.pre_grasp_posture.points.effort';
info.MatPath{139} = 'action_result.result.grasps.pre_grasp_posture.points.time_from_start';
info.MatPath{140} = 'action_result.result.grasps.pre_grasp_posture.points.time_from_start.sec';
info.MatPath{141} = 'action_result.result.grasps.pre_grasp_posture.points.time_from_start.nsec';
info.MatPath{142} = 'action_result.result.grasps.grasp_posture';
info.MatPath{143} = 'action_result.result.grasps.grasp_posture.header';
info.MatPath{144} = 'action_result.result.grasps.grasp_posture.header.seq';
info.MatPath{145} = 'action_result.result.grasps.grasp_posture.header.stamp';
info.MatPath{146} = 'action_result.result.grasps.grasp_posture.header.stamp.sec';
info.MatPath{147} = 'action_result.result.grasps.grasp_posture.header.stamp.nsec';
info.MatPath{148} = 'action_result.result.grasps.grasp_posture.header.frame_id';
info.MatPath{149} = 'action_result.result.grasps.grasp_posture.joint_names';
info.MatPath{150} = 'action_result.result.grasps.grasp_posture.points';
info.MatPath{151} = 'action_result.result.grasps.grasp_posture.points.positions';
info.MatPath{152} = 'action_result.result.grasps.grasp_posture.points.velocities';
info.MatPath{153} = 'action_result.result.grasps.grasp_posture.points.accelerations';
info.MatPath{154} = 'action_result.result.grasps.grasp_posture.points.effort';
info.MatPath{155} = 'action_result.result.grasps.grasp_posture.points.time_from_start';
info.MatPath{156} = 'action_result.result.grasps.grasp_posture.points.time_from_start.sec';
info.MatPath{157} = 'action_result.result.grasps.grasp_posture.points.time_from_start.nsec';
info.MatPath{158} = 'action_result.result.grasps.grasp_pose';
info.MatPath{159} = 'action_result.result.grasps.grasp_pose.header';
info.MatPath{160} = 'action_result.result.grasps.grasp_pose.header.seq';
info.MatPath{161} = 'action_result.result.grasps.grasp_pose.header.stamp';
info.MatPath{162} = 'action_result.result.grasps.grasp_pose.header.stamp.sec';
info.MatPath{163} = 'action_result.result.grasps.grasp_pose.header.stamp.nsec';
info.MatPath{164} = 'action_result.result.grasps.grasp_pose.header.frame_id';
info.MatPath{165} = 'action_result.result.grasps.grasp_pose.pose';
info.MatPath{166} = 'action_result.result.grasps.grasp_pose.pose.position';
info.MatPath{167} = 'action_result.result.grasps.grasp_pose.pose.position.x';
info.MatPath{168} = 'action_result.result.grasps.grasp_pose.pose.position.y';
info.MatPath{169} = 'action_result.result.grasps.grasp_pose.pose.position.z';
info.MatPath{170} = 'action_result.result.grasps.grasp_pose.pose.orientation';
info.MatPath{171} = 'action_result.result.grasps.grasp_pose.pose.orientation.x';
info.MatPath{172} = 'action_result.result.grasps.grasp_pose.pose.orientation.y';
info.MatPath{173} = 'action_result.result.grasps.grasp_pose.pose.orientation.z';
info.MatPath{174} = 'action_result.result.grasps.grasp_pose.pose.orientation.w';
info.MatPath{175} = 'action_result.result.grasps.grasp_quality';
info.MatPath{176} = 'action_result.result.grasps.pre_grasp_approach';
info.MatPath{177} = 'action_result.result.grasps.pre_grasp_approach.direction';
info.MatPath{178} = 'action_result.result.grasps.pre_grasp_approach.direction.header';
info.MatPath{179} = 'action_result.result.grasps.pre_grasp_approach.direction.header.seq';
info.MatPath{180} = 'action_result.result.grasps.pre_grasp_approach.direction.header.stamp';
info.MatPath{181} = 'action_result.result.grasps.pre_grasp_approach.direction.header.stamp.sec';
info.MatPath{182} = 'action_result.result.grasps.pre_grasp_approach.direction.header.stamp.nsec';
info.MatPath{183} = 'action_result.result.grasps.pre_grasp_approach.direction.header.frame_id';
info.MatPath{184} = 'action_result.result.grasps.pre_grasp_approach.direction.vector';
info.MatPath{185} = 'action_result.result.grasps.pre_grasp_approach.direction.vector.x';
info.MatPath{186} = 'action_result.result.grasps.pre_grasp_approach.direction.vector.y';
info.MatPath{187} = 'action_result.result.grasps.pre_grasp_approach.direction.vector.z';
info.MatPath{188} = 'action_result.result.grasps.pre_grasp_approach.desired_distance';
info.MatPath{189} = 'action_result.result.grasps.pre_grasp_approach.min_distance';
info.MatPath{190} = 'action_result.result.grasps.post_grasp_retreat';
info.MatPath{191} = 'action_result.result.grasps.post_grasp_retreat.direction';
info.MatPath{192} = 'action_result.result.grasps.post_grasp_retreat.direction.header';
info.MatPath{193} = 'action_result.result.grasps.post_grasp_retreat.direction.header.seq';
info.MatPath{194} = 'action_result.result.grasps.post_grasp_retreat.direction.header.stamp';
info.MatPath{195} = 'action_result.result.grasps.post_grasp_retreat.direction.header.stamp.sec';
info.MatPath{196} = 'action_result.result.grasps.post_grasp_retreat.direction.header.stamp.nsec';
info.MatPath{197} = 'action_result.result.grasps.post_grasp_retreat.direction.header.frame_id';
info.MatPath{198} = 'action_result.result.grasps.post_grasp_retreat.direction.vector';
info.MatPath{199} = 'action_result.result.grasps.post_grasp_retreat.direction.vector.x';
info.MatPath{200} = 'action_result.result.grasps.post_grasp_retreat.direction.vector.y';
info.MatPath{201} = 'action_result.result.grasps.post_grasp_retreat.direction.vector.z';
info.MatPath{202} = 'action_result.result.grasps.post_grasp_retreat.desired_distance';
info.MatPath{203} = 'action_result.result.grasps.post_grasp_retreat.min_distance';
info.MatPath{204} = 'action_result.result.grasps.post_place_retreat';
info.MatPath{205} = 'action_result.result.grasps.post_place_retreat.direction';
info.MatPath{206} = 'action_result.result.grasps.post_place_retreat.direction.header';
info.MatPath{207} = 'action_result.result.grasps.post_place_retreat.direction.header.seq';
info.MatPath{208} = 'action_result.result.grasps.post_place_retreat.direction.header.stamp';
info.MatPath{209} = 'action_result.result.grasps.post_place_retreat.direction.header.stamp.sec';
info.MatPath{210} = 'action_result.result.grasps.post_place_retreat.direction.header.stamp.nsec';
info.MatPath{211} = 'action_result.result.grasps.post_place_retreat.direction.header.frame_id';
info.MatPath{212} = 'action_result.result.grasps.post_place_retreat.direction.vector';
info.MatPath{213} = 'action_result.result.grasps.post_place_retreat.direction.vector.x';
info.MatPath{214} = 'action_result.result.grasps.post_place_retreat.direction.vector.y';
info.MatPath{215} = 'action_result.result.grasps.post_place_retreat.direction.vector.z';
info.MatPath{216} = 'action_result.result.grasps.post_place_retreat.desired_distance';
info.MatPath{217} = 'action_result.result.grasps.post_place_retreat.min_distance';
info.MatPath{218} = 'action_result.result.grasps.max_contact_force';
info.MatPath{219} = 'action_result.result.grasps.allowed_touch_objects';
info.MatPath{220} = 'action_feedback';
info.MatPath{221} = 'action_feedback.header';
info.MatPath{222} = 'action_feedback.header.seq';
info.MatPath{223} = 'action_feedback.header.stamp';
info.MatPath{224} = 'action_feedback.header.stamp.sec';
info.MatPath{225} = 'action_feedback.header.stamp.nsec';
info.MatPath{226} = 'action_feedback.header.frame_id';
info.MatPath{227} = 'action_feedback.status';
info.MatPath{228} = 'action_feedback.status.goal_id';
info.MatPath{229} = 'action_feedback.status.goal_id.stamp';
info.MatPath{230} = 'action_feedback.status.goal_id.stamp.sec';
info.MatPath{231} = 'action_feedback.status.goal_id.stamp.nsec';
info.MatPath{232} = 'action_feedback.status.goal_id.id';
info.MatPath{233} = 'action_feedback.status.status';
info.MatPath{234} = 'action_feedback.status.PENDING';
info.MatPath{235} = 'action_feedback.status.ACTIVE';
info.MatPath{236} = 'action_feedback.status.PREEMPTED';
info.MatPath{237} = 'action_feedback.status.SUCCEEDED';
info.MatPath{238} = 'action_feedback.status.ABORTED';
info.MatPath{239} = 'action_feedback.status.REJECTED';
info.MatPath{240} = 'action_feedback.status.PREEMPTING';
info.MatPath{241} = 'action_feedback.status.RECALLING';
info.MatPath{242} = 'action_feedback.status.RECALLED';
info.MatPath{243} = 'action_feedback.status.LOST';
info.MatPath{244} = 'action_feedback.status.text';
info.MatPath{245} = 'action_feedback.feedback';
info.MatPath{246} = 'action_feedback.feedback.grasps';
info.MatPath{247} = 'action_feedback.feedback.grasps.id';
info.MatPath{248} = 'action_feedback.feedback.grasps.pre_grasp_posture';
info.MatPath{249} = 'action_feedback.feedback.grasps.pre_grasp_posture.header';
info.MatPath{250} = 'action_feedback.feedback.grasps.pre_grasp_posture.header.seq';
info.MatPath{251} = 'action_feedback.feedback.grasps.pre_grasp_posture.header.stamp';
info.MatPath{252} = 'action_feedback.feedback.grasps.pre_grasp_posture.header.stamp.sec';
info.MatPath{253} = 'action_feedback.feedback.grasps.pre_grasp_posture.header.stamp.nsec';
info.MatPath{254} = 'action_feedback.feedback.grasps.pre_grasp_posture.header.frame_id';
info.MatPath{255} = 'action_feedback.feedback.grasps.pre_grasp_posture.joint_names';
info.MatPath{256} = 'action_feedback.feedback.grasps.pre_grasp_posture.points';
info.MatPath{257} = 'action_feedback.feedback.grasps.pre_grasp_posture.points.positions';
info.MatPath{258} = 'action_feedback.feedback.grasps.pre_grasp_posture.points.velocities';
info.MatPath{259} = 'action_feedback.feedback.grasps.pre_grasp_posture.points.accelerations';
info.MatPath{260} = 'action_feedback.feedback.grasps.pre_grasp_posture.points.effort';
info.MatPath{261} = 'action_feedback.feedback.grasps.pre_grasp_posture.points.time_from_start';
info.MatPath{262} = 'action_feedback.feedback.grasps.pre_grasp_posture.points.time_from_start.sec';
info.MatPath{263} = 'action_feedback.feedback.grasps.pre_grasp_posture.points.time_from_start.nsec';
info.MatPath{264} = 'action_feedback.feedback.grasps.grasp_posture';
info.MatPath{265} = 'action_feedback.feedback.grasps.grasp_posture.header';
info.MatPath{266} = 'action_feedback.feedback.grasps.grasp_posture.header.seq';
info.MatPath{267} = 'action_feedback.feedback.grasps.grasp_posture.header.stamp';
info.MatPath{268} = 'action_feedback.feedback.grasps.grasp_posture.header.stamp.sec';
info.MatPath{269} = 'action_feedback.feedback.grasps.grasp_posture.header.stamp.nsec';
info.MatPath{270} = 'action_feedback.feedback.grasps.grasp_posture.header.frame_id';
info.MatPath{271} = 'action_feedback.feedback.grasps.grasp_posture.joint_names';
info.MatPath{272} = 'action_feedback.feedback.grasps.grasp_posture.points';
info.MatPath{273} = 'action_feedback.feedback.grasps.grasp_posture.points.positions';
info.MatPath{274} = 'action_feedback.feedback.grasps.grasp_posture.points.velocities';
info.MatPath{275} = 'action_feedback.feedback.grasps.grasp_posture.points.accelerations';
info.MatPath{276} = 'action_feedback.feedback.grasps.grasp_posture.points.effort';
info.MatPath{277} = 'action_feedback.feedback.grasps.grasp_posture.points.time_from_start';
info.MatPath{278} = 'action_feedback.feedback.grasps.grasp_posture.points.time_from_start.sec';
info.MatPath{279} = 'action_feedback.feedback.grasps.grasp_posture.points.time_from_start.nsec';
info.MatPath{280} = 'action_feedback.feedback.grasps.grasp_pose';
info.MatPath{281} = 'action_feedback.feedback.grasps.grasp_pose.header';
info.MatPath{282} = 'action_feedback.feedback.grasps.grasp_pose.header.seq';
info.MatPath{283} = 'action_feedback.feedback.grasps.grasp_pose.header.stamp';
info.MatPath{284} = 'action_feedback.feedback.grasps.grasp_pose.header.stamp.sec';
info.MatPath{285} = 'action_feedback.feedback.grasps.grasp_pose.header.stamp.nsec';
info.MatPath{286} = 'action_feedback.feedback.grasps.grasp_pose.header.frame_id';
info.MatPath{287} = 'action_feedback.feedback.grasps.grasp_pose.pose';
info.MatPath{288} = 'action_feedback.feedback.grasps.grasp_pose.pose.position';
info.MatPath{289} = 'action_feedback.feedback.grasps.grasp_pose.pose.position.x';
info.MatPath{290} = 'action_feedback.feedback.grasps.grasp_pose.pose.position.y';
info.MatPath{291} = 'action_feedback.feedback.grasps.grasp_pose.pose.position.z';
info.MatPath{292} = 'action_feedback.feedback.grasps.grasp_pose.pose.orientation';
info.MatPath{293} = 'action_feedback.feedback.grasps.grasp_pose.pose.orientation.x';
info.MatPath{294} = 'action_feedback.feedback.grasps.grasp_pose.pose.orientation.y';
info.MatPath{295} = 'action_feedback.feedback.grasps.grasp_pose.pose.orientation.z';
info.MatPath{296} = 'action_feedback.feedback.grasps.grasp_pose.pose.orientation.w';
info.MatPath{297} = 'action_feedback.feedback.grasps.grasp_quality';
info.MatPath{298} = 'action_feedback.feedback.grasps.pre_grasp_approach';
info.MatPath{299} = 'action_feedback.feedback.grasps.pre_grasp_approach.direction';
info.MatPath{300} = 'action_feedback.feedback.grasps.pre_grasp_approach.direction.header';
info.MatPath{301} = 'action_feedback.feedback.grasps.pre_grasp_approach.direction.header.seq';
info.MatPath{302} = 'action_feedback.feedback.grasps.pre_grasp_approach.direction.header.stamp';
info.MatPath{303} = 'action_feedback.feedback.grasps.pre_grasp_approach.direction.header.stamp.sec';
info.MatPath{304} = 'action_feedback.feedback.grasps.pre_grasp_approach.direction.header.stamp.nsec';
info.MatPath{305} = 'action_feedback.feedback.grasps.pre_grasp_approach.direction.header.frame_id';
info.MatPath{306} = 'action_feedback.feedback.grasps.pre_grasp_approach.direction.vector';
info.MatPath{307} = 'action_feedback.feedback.grasps.pre_grasp_approach.direction.vector.x';
info.MatPath{308} = 'action_feedback.feedback.grasps.pre_grasp_approach.direction.vector.y';
info.MatPath{309} = 'action_feedback.feedback.grasps.pre_grasp_approach.direction.vector.z';
info.MatPath{310} = 'action_feedback.feedback.grasps.pre_grasp_approach.desired_distance';
info.MatPath{311} = 'action_feedback.feedback.grasps.pre_grasp_approach.min_distance';
info.MatPath{312} = 'action_feedback.feedback.grasps.post_grasp_retreat';
info.MatPath{313} = 'action_feedback.feedback.grasps.post_grasp_retreat.direction';
info.MatPath{314} = 'action_feedback.feedback.grasps.post_grasp_retreat.direction.header';
info.MatPath{315} = 'action_feedback.feedback.grasps.post_grasp_retreat.direction.header.seq';
info.MatPath{316} = 'action_feedback.feedback.grasps.post_grasp_retreat.direction.header.stamp';
info.MatPath{317} = 'action_feedback.feedback.grasps.post_grasp_retreat.direction.header.stamp.sec';
info.MatPath{318} = 'action_feedback.feedback.grasps.post_grasp_retreat.direction.header.stamp.nsec';
info.MatPath{319} = 'action_feedback.feedback.grasps.post_grasp_retreat.direction.header.frame_id';
info.MatPath{320} = 'action_feedback.feedback.grasps.post_grasp_retreat.direction.vector';
info.MatPath{321} = 'action_feedback.feedback.grasps.post_grasp_retreat.direction.vector.x';
info.MatPath{322} = 'action_feedback.feedback.grasps.post_grasp_retreat.direction.vector.y';
info.MatPath{323} = 'action_feedback.feedback.grasps.post_grasp_retreat.direction.vector.z';
info.MatPath{324} = 'action_feedback.feedback.grasps.post_grasp_retreat.desired_distance';
info.MatPath{325} = 'action_feedback.feedback.grasps.post_grasp_retreat.min_distance';
info.MatPath{326} = 'action_feedback.feedback.grasps.post_place_retreat';
info.MatPath{327} = 'action_feedback.feedback.grasps.post_place_retreat.direction';
info.MatPath{328} = 'action_feedback.feedback.grasps.post_place_retreat.direction.header';
info.MatPath{329} = 'action_feedback.feedback.grasps.post_place_retreat.direction.header.seq';
info.MatPath{330} = 'action_feedback.feedback.grasps.post_place_retreat.direction.header.stamp';
info.MatPath{331} = 'action_feedback.feedback.grasps.post_place_retreat.direction.header.stamp.sec';
info.MatPath{332} = 'action_feedback.feedback.grasps.post_place_retreat.direction.header.stamp.nsec';
info.MatPath{333} = 'action_feedback.feedback.grasps.post_place_retreat.direction.header.frame_id';
info.MatPath{334} = 'action_feedback.feedback.grasps.post_place_retreat.direction.vector';
info.MatPath{335} = 'action_feedback.feedback.grasps.post_place_retreat.direction.vector.x';
info.MatPath{336} = 'action_feedback.feedback.grasps.post_place_retreat.direction.vector.y';
info.MatPath{337} = 'action_feedback.feedback.grasps.post_place_retreat.direction.vector.z';
info.MatPath{338} = 'action_feedback.feedback.grasps.post_place_retreat.desired_distance';
info.MatPath{339} = 'action_feedback.feedback.grasps.post_place_retreat.min_distance';
info.MatPath{340} = 'action_feedback.feedback.grasps.max_contact_force';
info.MatPath{341} = 'action_feedback.feedback.grasps.allowed_touch_objects';