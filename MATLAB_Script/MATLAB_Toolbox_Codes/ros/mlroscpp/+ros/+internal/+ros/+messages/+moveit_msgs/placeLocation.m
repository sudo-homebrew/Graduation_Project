function [data, info] = placeLocation
%PlaceLocation gives an empty data for moveit_msgs/PlaceLocation

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'moveit_msgs/PlaceLocation';
[data.Id, info.Id] = ros.internal.ros.messages.ros.char('string',0);
[data.PostPlacePosture, info.PostPlacePosture] = ros.internal.ros.messages.trajectory_msgs.jointTrajectory;
info.PostPlacePosture.MLdataType = 'struct';
[data.PlacePose, info.PlacePose] = ros.internal.ros.messages.geometry_msgs.poseStamped;
info.PlacePose.MLdataType = 'struct';
[data.Quality, info.Quality] = ros.internal.ros.messages.ros.default_type('double',1);
[data.PrePlaceApproach, info.PrePlaceApproach] = ros.internal.ros.messages.moveit_msgs.gripperTranslation;
info.PrePlaceApproach.MLdataType = 'struct';
[data.PostPlaceRetreat, info.PostPlaceRetreat] = ros.internal.ros.messages.moveit_msgs.gripperTranslation;
info.PostPlaceRetreat.MLdataType = 'struct';
[data.AllowedTouchObjects, info.AllowedTouchObjects] = ros.internal.ros.messages.ros.char('string',NaN);
info.MessageType = 'moveit_msgs/PlaceLocation';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,64);
info.MatPath{1} = 'id';
info.MatPath{2} = 'post_place_posture';
info.MatPath{3} = 'post_place_posture.header';
info.MatPath{4} = 'post_place_posture.header.seq';
info.MatPath{5} = 'post_place_posture.header.stamp';
info.MatPath{6} = 'post_place_posture.header.stamp.sec';
info.MatPath{7} = 'post_place_posture.header.stamp.nsec';
info.MatPath{8} = 'post_place_posture.header.frame_id';
info.MatPath{9} = 'post_place_posture.joint_names';
info.MatPath{10} = 'post_place_posture.points';
info.MatPath{11} = 'post_place_posture.points.positions';
info.MatPath{12} = 'post_place_posture.points.velocities';
info.MatPath{13} = 'post_place_posture.points.accelerations';
info.MatPath{14} = 'post_place_posture.points.effort';
info.MatPath{15} = 'post_place_posture.points.time_from_start';
info.MatPath{16} = 'post_place_posture.points.time_from_start.sec';
info.MatPath{17} = 'post_place_posture.points.time_from_start.nsec';
info.MatPath{18} = 'place_pose';
info.MatPath{19} = 'place_pose.header';
info.MatPath{20} = 'place_pose.header.seq';
info.MatPath{21} = 'place_pose.header.stamp';
info.MatPath{22} = 'place_pose.header.stamp.sec';
info.MatPath{23} = 'place_pose.header.stamp.nsec';
info.MatPath{24} = 'place_pose.header.frame_id';
info.MatPath{25} = 'place_pose.pose';
info.MatPath{26} = 'place_pose.pose.position';
info.MatPath{27} = 'place_pose.pose.position.x';
info.MatPath{28} = 'place_pose.pose.position.y';
info.MatPath{29} = 'place_pose.pose.position.z';
info.MatPath{30} = 'place_pose.pose.orientation';
info.MatPath{31} = 'place_pose.pose.orientation.x';
info.MatPath{32} = 'place_pose.pose.orientation.y';
info.MatPath{33} = 'place_pose.pose.orientation.z';
info.MatPath{34} = 'place_pose.pose.orientation.w';
info.MatPath{35} = 'quality';
info.MatPath{36} = 'pre_place_approach';
info.MatPath{37} = 'pre_place_approach.direction';
info.MatPath{38} = 'pre_place_approach.direction.header';
info.MatPath{39} = 'pre_place_approach.direction.header.seq';
info.MatPath{40} = 'pre_place_approach.direction.header.stamp';
info.MatPath{41} = 'pre_place_approach.direction.header.stamp.sec';
info.MatPath{42} = 'pre_place_approach.direction.header.stamp.nsec';
info.MatPath{43} = 'pre_place_approach.direction.header.frame_id';
info.MatPath{44} = 'pre_place_approach.direction.vector';
info.MatPath{45} = 'pre_place_approach.direction.vector.x';
info.MatPath{46} = 'pre_place_approach.direction.vector.y';
info.MatPath{47} = 'pre_place_approach.direction.vector.z';
info.MatPath{48} = 'pre_place_approach.desired_distance';
info.MatPath{49} = 'pre_place_approach.min_distance';
info.MatPath{50} = 'post_place_retreat';
info.MatPath{51} = 'post_place_retreat.direction';
info.MatPath{52} = 'post_place_retreat.direction.header';
info.MatPath{53} = 'post_place_retreat.direction.header.seq';
info.MatPath{54} = 'post_place_retreat.direction.header.stamp';
info.MatPath{55} = 'post_place_retreat.direction.header.stamp.sec';
info.MatPath{56} = 'post_place_retreat.direction.header.stamp.nsec';
info.MatPath{57} = 'post_place_retreat.direction.header.frame_id';
info.MatPath{58} = 'post_place_retreat.direction.vector';
info.MatPath{59} = 'post_place_retreat.direction.vector.x';
info.MatPath{60} = 'post_place_retreat.direction.vector.y';
info.MatPath{61} = 'post_place_retreat.direction.vector.z';
info.MatPath{62} = 'post_place_retreat.desired_distance';
info.MatPath{63} = 'post_place_retreat.min_distance';
info.MatPath{64} = 'allowed_touch_objects';