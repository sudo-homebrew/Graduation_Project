function [data, info] = executeTrajectoryActionGoal
%ExecuteTrajectoryActionGoal gives an empty data for moveit_msgs/ExecuteTrajectoryActionGoal

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'moveit_msgs/ExecuteTrajectoryActionGoal';
[data.Header, info.Header] = ros.internal.ros.messages.std_msgs.header;
info.Header.MLdataType = 'struct';
[data.GoalId, info.GoalId] = ros.internal.ros.messages.actionlib_msgs.goalID;
info.GoalId.MLdataType = 'struct';
[data.Goal, info.Goal] = ros.internal.ros.messages.moveit_msgs.executeTrajectoryGoal;
info.Goal.MLdataType = 'struct';
info.MessageType = 'moveit_msgs/ExecuteTrajectoryActionGoal';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,69);
info.MatPath{1} = 'header';
info.MatPath{2} = 'header.seq';
info.MatPath{3} = 'header.stamp';
info.MatPath{4} = 'header.stamp.sec';
info.MatPath{5} = 'header.stamp.nsec';
info.MatPath{6} = 'header.frame_id';
info.MatPath{7} = 'goal_id';
info.MatPath{8} = 'goal_id.stamp';
info.MatPath{9} = 'goal_id.stamp.sec';
info.MatPath{10} = 'goal_id.stamp.nsec';
info.MatPath{11} = 'goal_id.id';
info.MatPath{12} = 'goal';
info.MatPath{13} = 'goal.trajectory';
info.MatPath{14} = 'goal.trajectory.joint_trajectory';
info.MatPath{15} = 'goal.trajectory.joint_trajectory.header';
info.MatPath{16} = 'goal.trajectory.joint_trajectory.header.seq';
info.MatPath{17} = 'goal.trajectory.joint_trajectory.header.stamp';
info.MatPath{18} = 'goal.trajectory.joint_trajectory.header.stamp.sec';
info.MatPath{19} = 'goal.trajectory.joint_trajectory.header.stamp.nsec';
info.MatPath{20} = 'goal.trajectory.joint_trajectory.header.frame_id';
info.MatPath{21} = 'goal.trajectory.joint_trajectory.joint_names';
info.MatPath{22} = 'goal.trajectory.joint_trajectory.points';
info.MatPath{23} = 'goal.trajectory.joint_trajectory.points.positions';
info.MatPath{24} = 'goal.trajectory.joint_trajectory.points.velocities';
info.MatPath{25} = 'goal.trajectory.joint_trajectory.points.accelerations';
info.MatPath{26} = 'goal.trajectory.joint_trajectory.points.effort';
info.MatPath{27} = 'goal.trajectory.joint_trajectory.points.time_from_start';
info.MatPath{28} = 'goal.trajectory.joint_trajectory.points.time_from_start.sec';
info.MatPath{29} = 'goal.trajectory.joint_trajectory.points.time_from_start.nsec';
info.MatPath{30} = 'goal.trajectory.multi_dof_joint_trajectory';
info.MatPath{31} = 'goal.trajectory.multi_dof_joint_trajectory.header';
info.MatPath{32} = 'goal.trajectory.multi_dof_joint_trajectory.header.seq';
info.MatPath{33} = 'goal.trajectory.multi_dof_joint_trajectory.header.stamp';
info.MatPath{34} = 'goal.trajectory.multi_dof_joint_trajectory.header.stamp.sec';
info.MatPath{35} = 'goal.trajectory.multi_dof_joint_trajectory.header.stamp.nsec';
info.MatPath{36} = 'goal.trajectory.multi_dof_joint_trajectory.header.frame_id';
info.MatPath{37} = 'goal.trajectory.multi_dof_joint_trajectory.joint_names';
info.MatPath{38} = 'goal.trajectory.multi_dof_joint_trajectory.points';
info.MatPath{39} = 'goal.trajectory.multi_dof_joint_trajectory.points.transforms';
info.MatPath{40} = 'goal.trajectory.multi_dof_joint_trajectory.points.transforms.translation';
info.MatPath{41} = 'goal.trajectory.multi_dof_joint_trajectory.points.transforms.translation.x';
info.MatPath{42} = 'goal.trajectory.multi_dof_joint_trajectory.points.transforms.translation.y';
info.MatPath{43} = 'goal.trajectory.multi_dof_joint_trajectory.points.transforms.translation.z';
info.MatPath{44} = 'goal.trajectory.multi_dof_joint_trajectory.points.transforms.rotation';
info.MatPath{45} = 'goal.trajectory.multi_dof_joint_trajectory.points.transforms.rotation.x';
info.MatPath{46} = 'goal.trajectory.multi_dof_joint_trajectory.points.transforms.rotation.y';
info.MatPath{47} = 'goal.trajectory.multi_dof_joint_trajectory.points.transforms.rotation.z';
info.MatPath{48} = 'goal.trajectory.multi_dof_joint_trajectory.points.transforms.rotation.w';
info.MatPath{49} = 'goal.trajectory.multi_dof_joint_trajectory.points.velocities';
info.MatPath{50} = 'goal.trajectory.multi_dof_joint_trajectory.points.velocities.linear';
info.MatPath{51} = 'goal.trajectory.multi_dof_joint_trajectory.points.velocities.linear.x';
info.MatPath{52} = 'goal.trajectory.multi_dof_joint_trajectory.points.velocities.linear.y';
info.MatPath{53} = 'goal.trajectory.multi_dof_joint_trajectory.points.velocities.linear.z';
info.MatPath{54} = 'goal.trajectory.multi_dof_joint_trajectory.points.velocities.angular';
info.MatPath{55} = 'goal.trajectory.multi_dof_joint_trajectory.points.velocities.angular.x';
info.MatPath{56} = 'goal.trajectory.multi_dof_joint_trajectory.points.velocities.angular.y';
info.MatPath{57} = 'goal.trajectory.multi_dof_joint_trajectory.points.velocities.angular.z';
info.MatPath{58} = 'goal.trajectory.multi_dof_joint_trajectory.points.accelerations';
info.MatPath{59} = 'goal.trajectory.multi_dof_joint_trajectory.points.accelerations.linear';
info.MatPath{60} = 'goal.trajectory.multi_dof_joint_trajectory.points.accelerations.linear.x';
info.MatPath{61} = 'goal.trajectory.multi_dof_joint_trajectory.points.accelerations.linear.y';
info.MatPath{62} = 'goal.trajectory.multi_dof_joint_trajectory.points.accelerations.linear.z';
info.MatPath{63} = 'goal.trajectory.multi_dof_joint_trajectory.points.accelerations.angular';
info.MatPath{64} = 'goal.trajectory.multi_dof_joint_trajectory.points.accelerations.angular.x';
info.MatPath{65} = 'goal.trajectory.multi_dof_joint_trajectory.points.accelerations.angular.y';
info.MatPath{66} = 'goal.trajectory.multi_dof_joint_trajectory.points.accelerations.angular.z';
info.MatPath{67} = 'goal.trajectory.multi_dof_joint_trajectory.points.time_from_start';
info.MatPath{68} = 'goal.trajectory.multi_dof_joint_trajectory.points.time_from_start.sec';
info.MatPath{69} = 'goal.trajectory.multi_dof_joint_trajectory.points.time_from_start.nsec';