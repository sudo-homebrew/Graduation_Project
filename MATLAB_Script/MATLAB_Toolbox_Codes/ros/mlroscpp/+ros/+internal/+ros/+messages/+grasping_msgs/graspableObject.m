function [data, info] = graspableObject
%GraspableObject gives an empty data for grasping_msgs/GraspableObject

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'grasping_msgs/GraspableObject';
[data.Object, info.Object] = ros.internal.ros.messages.grasping_msgs.object;
info.Object.MLdataType = 'struct';
[data.Grasps, info.Grasps] = ros.internal.ros.messages.moveit_msgs.grasp;
info.Grasps.MLdataType = 'struct';
info.Grasps.MaxLen = NaN;
info.Grasps.MinLen = 0;
data.Grasps = data.Grasps([],1);
info.MessageType = 'grasping_msgs/GraspableObject';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,179);
info.MatPath{1} = 'object';
info.MatPath{2} = 'object.header';
info.MatPath{3} = 'object.header.seq';
info.MatPath{4} = 'object.header.stamp';
info.MatPath{5} = 'object.header.stamp.sec';
info.MatPath{6} = 'object.header.stamp.nsec';
info.MatPath{7} = 'object.header.frame_id';
info.MatPath{8} = 'object.name';
info.MatPath{9} = 'object.support_surface';
info.MatPath{10} = 'object.properties';
info.MatPath{11} = 'object.properties.name';
info.MatPath{12} = 'object.properties.value';
info.MatPath{13} = 'object.point_cluster';
info.MatPath{14} = 'object.point_cluster.header';
info.MatPath{15} = 'object.point_cluster.header.seq';
info.MatPath{16} = 'object.point_cluster.header.stamp';
info.MatPath{17} = 'object.point_cluster.header.stamp.sec';
info.MatPath{18} = 'object.point_cluster.header.stamp.nsec';
info.MatPath{19} = 'object.point_cluster.header.frame_id';
info.MatPath{20} = 'object.point_cluster.height';
info.MatPath{21} = 'object.point_cluster.width';
info.MatPath{22} = 'object.point_cluster.fields';
info.MatPath{23} = 'object.point_cluster.fields.INT8';
info.MatPath{24} = 'object.point_cluster.fields.UINT8';
info.MatPath{25} = 'object.point_cluster.fields.INT16';
info.MatPath{26} = 'object.point_cluster.fields.UINT16';
info.MatPath{27} = 'object.point_cluster.fields.INT32';
info.MatPath{28} = 'object.point_cluster.fields.UINT32';
info.MatPath{29} = 'object.point_cluster.fields.FLOAT32';
info.MatPath{30} = 'object.point_cluster.fields.FLOAT64';
info.MatPath{31} = 'object.point_cluster.fields.name';
info.MatPath{32} = 'object.point_cluster.fields.offset';
info.MatPath{33} = 'object.point_cluster.fields.datatype';
info.MatPath{34} = 'object.point_cluster.fields.count';
info.MatPath{35} = 'object.point_cluster.is_bigendian';
info.MatPath{36} = 'object.point_cluster.point_step';
info.MatPath{37} = 'object.point_cluster.row_step';
info.MatPath{38} = 'object.point_cluster.data';
info.MatPath{39} = 'object.point_cluster.is_dense';
info.MatPath{40} = 'object.primitives';
info.MatPath{41} = 'object.primitives.BOX';
info.MatPath{42} = 'object.primitives.SPHERE';
info.MatPath{43} = 'object.primitives.CYLINDER';
info.MatPath{44} = 'object.primitives.CONE';
info.MatPath{45} = 'object.primitives.type';
info.MatPath{46} = 'object.primitives.dimensions';
info.MatPath{47} = 'object.primitives.BOX_X';
info.MatPath{48} = 'object.primitives.BOX_Y';
info.MatPath{49} = 'object.primitives.BOX_Z';
info.MatPath{50} = 'object.primitives.SPHERE_RADIUS';
info.MatPath{51} = 'object.primitives.CYLINDER_HEIGHT';
info.MatPath{52} = 'object.primitives.CYLINDER_RADIUS';
info.MatPath{53} = 'object.primitives.CONE_HEIGHT';
info.MatPath{54} = 'object.primitives.CONE_RADIUS';
info.MatPath{55} = 'object.primitive_poses';
info.MatPath{56} = 'object.primitive_poses.position';
info.MatPath{57} = 'object.primitive_poses.position.x';
info.MatPath{58} = 'object.primitive_poses.position.y';
info.MatPath{59} = 'object.primitive_poses.position.z';
info.MatPath{60} = 'object.primitive_poses.orientation';
info.MatPath{61} = 'object.primitive_poses.orientation.x';
info.MatPath{62} = 'object.primitive_poses.orientation.y';
info.MatPath{63} = 'object.primitive_poses.orientation.z';
info.MatPath{64} = 'object.primitive_poses.orientation.w';
info.MatPath{65} = 'object.meshes';
info.MatPath{66} = 'object.meshes.triangles';
info.MatPath{67} = 'object.meshes.triangles.vertex_indices';
info.MatPath{68} = 'object.meshes.vertices';
info.MatPath{69} = 'object.meshes.vertices.x';
info.MatPath{70} = 'object.meshes.vertices.y';
info.MatPath{71} = 'object.meshes.vertices.z';
info.MatPath{72} = 'object.mesh_poses';
info.MatPath{73} = 'object.mesh_poses.position';
info.MatPath{74} = 'object.mesh_poses.position.x';
info.MatPath{75} = 'object.mesh_poses.position.y';
info.MatPath{76} = 'object.mesh_poses.position.z';
info.MatPath{77} = 'object.mesh_poses.orientation';
info.MatPath{78} = 'object.mesh_poses.orientation.x';
info.MatPath{79} = 'object.mesh_poses.orientation.y';
info.MatPath{80} = 'object.mesh_poses.orientation.z';
info.MatPath{81} = 'object.mesh_poses.orientation.w';
info.MatPath{82} = 'object.surface';
info.MatPath{83} = 'object.surface.coef';
info.MatPath{84} = 'grasps';
info.MatPath{85} = 'grasps.id';
info.MatPath{86} = 'grasps.pre_grasp_posture';
info.MatPath{87} = 'grasps.pre_grasp_posture.header';
info.MatPath{88} = 'grasps.pre_grasp_posture.header.seq';
info.MatPath{89} = 'grasps.pre_grasp_posture.header.stamp';
info.MatPath{90} = 'grasps.pre_grasp_posture.header.stamp.sec';
info.MatPath{91} = 'grasps.pre_grasp_posture.header.stamp.nsec';
info.MatPath{92} = 'grasps.pre_grasp_posture.header.frame_id';
info.MatPath{93} = 'grasps.pre_grasp_posture.joint_names';
info.MatPath{94} = 'grasps.pre_grasp_posture.points';
info.MatPath{95} = 'grasps.pre_grasp_posture.points.positions';
info.MatPath{96} = 'grasps.pre_grasp_posture.points.velocities';
info.MatPath{97} = 'grasps.pre_grasp_posture.points.accelerations';
info.MatPath{98} = 'grasps.pre_grasp_posture.points.effort';
info.MatPath{99} = 'grasps.pre_grasp_posture.points.time_from_start';
info.MatPath{100} = 'grasps.pre_grasp_posture.points.time_from_start.sec';
info.MatPath{101} = 'grasps.pre_grasp_posture.points.time_from_start.nsec';
info.MatPath{102} = 'grasps.grasp_posture';
info.MatPath{103} = 'grasps.grasp_posture.header';
info.MatPath{104} = 'grasps.grasp_posture.header.seq';
info.MatPath{105} = 'grasps.grasp_posture.header.stamp';
info.MatPath{106} = 'grasps.grasp_posture.header.stamp.sec';
info.MatPath{107} = 'grasps.grasp_posture.header.stamp.nsec';
info.MatPath{108} = 'grasps.grasp_posture.header.frame_id';
info.MatPath{109} = 'grasps.grasp_posture.joint_names';
info.MatPath{110} = 'grasps.grasp_posture.points';
info.MatPath{111} = 'grasps.grasp_posture.points.positions';
info.MatPath{112} = 'grasps.grasp_posture.points.velocities';
info.MatPath{113} = 'grasps.grasp_posture.points.accelerations';
info.MatPath{114} = 'grasps.grasp_posture.points.effort';
info.MatPath{115} = 'grasps.grasp_posture.points.time_from_start';
info.MatPath{116} = 'grasps.grasp_posture.points.time_from_start.sec';
info.MatPath{117} = 'grasps.grasp_posture.points.time_from_start.nsec';
info.MatPath{118} = 'grasps.grasp_pose';
info.MatPath{119} = 'grasps.grasp_pose.header';
info.MatPath{120} = 'grasps.grasp_pose.header.seq';
info.MatPath{121} = 'grasps.grasp_pose.header.stamp';
info.MatPath{122} = 'grasps.grasp_pose.header.stamp.sec';
info.MatPath{123} = 'grasps.grasp_pose.header.stamp.nsec';
info.MatPath{124} = 'grasps.grasp_pose.header.frame_id';
info.MatPath{125} = 'grasps.grasp_pose.pose';
info.MatPath{126} = 'grasps.grasp_pose.pose.position';
info.MatPath{127} = 'grasps.grasp_pose.pose.position.x';
info.MatPath{128} = 'grasps.grasp_pose.pose.position.y';
info.MatPath{129} = 'grasps.grasp_pose.pose.position.z';
info.MatPath{130} = 'grasps.grasp_pose.pose.orientation';
info.MatPath{131} = 'grasps.grasp_pose.pose.orientation.x';
info.MatPath{132} = 'grasps.grasp_pose.pose.orientation.y';
info.MatPath{133} = 'grasps.grasp_pose.pose.orientation.z';
info.MatPath{134} = 'grasps.grasp_pose.pose.orientation.w';
info.MatPath{135} = 'grasps.grasp_quality';
info.MatPath{136} = 'grasps.pre_grasp_approach';
info.MatPath{137} = 'grasps.pre_grasp_approach.direction';
info.MatPath{138} = 'grasps.pre_grasp_approach.direction.header';
info.MatPath{139} = 'grasps.pre_grasp_approach.direction.header.seq';
info.MatPath{140} = 'grasps.pre_grasp_approach.direction.header.stamp';
info.MatPath{141} = 'grasps.pre_grasp_approach.direction.header.stamp.sec';
info.MatPath{142} = 'grasps.pre_grasp_approach.direction.header.stamp.nsec';
info.MatPath{143} = 'grasps.pre_grasp_approach.direction.header.frame_id';
info.MatPath{144} = 'grasps.pre_grasp_approach.direction.vector';
info.MatPath{145} = 'grasps.pre_grasp_approach.direction.vector.x';
info.MatPath{146} = 'grasps.pre_grasp_approach.direction.vector.y';
info.MatPath{147} = 'grasps.pre_grasp_approach.direction.vector.z';
info.MatPath{148} = 'grasps.pre_grasp_approach.desired_distance';
info.MatPath{149} = 'grasps.pre_grasp_approach.min_distance';
info.MatPath{150} = 'grasps.post_grasp_retreat';
info.MatPath{151} = 'grasps.post_grasp_retreat.direction';
info.MatPath{152} = 'grasps.post_grasp_retreat.direction.header';
info.MatPath{153} = 'grasps.post_grasp_retreat.direction.header.seq';
info.MatPath{154} = 'grasps.post_grasp_retreat.direction.header.stamp';
info.MatPath{155} = 'grasps.post_grasp_retreat.direction.header.stamp.sec';
info.MatPath{156} = 'grasps.post_grasp_retreat.direction.header.stamp.nsec';
info.MatPath{157} = 'grasps.post_grasp_retreat.direction.header.frame_id';
info.MatPath{158} = 'grasps.post_grasp_retreat.direction.vector';
info.MatPath{159} = 'grasps.post_grasp_retreat.direction.vector.x';
info.MatPath{160} = 'grasps.post_grasp_retreat.direction.vector.y';
info.MatPath{161} = 'grasps.post_grasp_retreat.direction.vector.z';
info.MatPath{162} = 'grasps.post_grasp_retreat.desired_distance';
info.MatPath{163} = 'grasps.post_grasp_retreat.min_distance';
info.MatPath{164} = 'grasps.post_place_retreat';
info.MatPath{165} = 'grasps.post_place_retreat.direction';
info.MatPath{166} = 'grasps.post_place_retreat.direction.header';
info.MatPath{167} = 'grasps.post_place_retreat.direction.header.seq';
info.MatPath{168} = 'grasps.post_place_retreat.direction.header.stamp';
info.MatPath{169} = 'grasps.post_place_retreat.direction.header.stamp.sec';
info.MatPath{170} = 'grasps.post_place_retreat.direction.header.stamp.nsec';
info.MatPath{171} = 'grasps.post_place_retreat.direction.header.frame_id';
info.MatPath{172} = 'grasps.post_place_retreat.direction.vector';
info.MatPath{173} = 'grasps.post_place_retreat.direction.vector.x';
info.MatPath{174} = 'grasps.post_place_retreat.direction.vector.y';
info.MatPath{175} = 'grasps.post_place_retreat.direction.vector.z';
info.MatPath{176} = 'grasps.post_place_retreat.desired_distance';
info.MatPath{177} = 'grasps.post_place_retreat.min_distance';
info.MatPath{178} = 'grasps.max_contact_force';
info.MatPath{179} = 'grasps.allowed_touch_objects';