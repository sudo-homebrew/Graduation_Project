function [data, info] = setJointTrajectoryRequest
%SetJointTrajectory gives an empty data for gazebo_msgs/SetJointTrajectoryRequest

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'gazebo_msgs/SetJointTrajectoryRequest';
[data.ModelName, info.ModelName] = ros.internal.ros.messages.ros.char('string',0);
[data.JointTrajectory, info.JointTrajectory] = ros.internal.ros.messages.trajectory_msgs.jointTrajectory;
info.JointTrajectory.MLdataType = 'struct';
[data.ModelPose, info.ModelPose] = ros.internal.ros.messages.geometry_msgs.pose;
info.ModelPose.MLdataType = 'struct';
[data.SetModelPose, info.SetModelPose] = ros.internal.ros.messages.ros.default_type('logical',1);
[data.DisablePhysicsUpdates, info.DisablePhysicsUpdates] = ros.internal.ros.messages.ros.default_type('logical',1);
info.MessageType = 'gazebo_msgs/SetJointTrajectoryRequest';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,29);
info.MatPath{1} = 'model_name';
info.MatPath{2} = 'joint_trajectory';
info.MatPath{3} = 'joint_trajectory.header';
info.MatPath{4} = 'joint_trajectory.header.seq';
info.MatPath{5} = 'joint_trajectory.header.stamp';
info.MatPath{6} = 'joint_trajectory.header.stamp.sec';
info.MatPath{7} = 'joint_trajectory.header.stamp.nsec';
info.MatPath{8} = 'joint_trajectory.header.frame_id';
info.MatPath{9} = 'joint_trajectory.joint_names';
info.MatPath{10} = 'joint_trajectory.points';
info.MatPath{11} = 'joint_trajectory.points.positions';
info.MatPath{12} = 'joint_trajectory.points.velocities';
info.MatPath{13} = 'joint_trajectory.points.accelerations';
info.MatPath{14} = 'joint_trajectory.points.effort';
info.MatPath{15} = 'joint_trajectory.points.time_from_start';
info.MatPath{16} = 'joint_trajectory.points.time_from_start.sec';
info.MatPath{17} = 'joint_trajectory.points.time_from_start.nsec';
info.MatPath{18} = 'model_pose';
info.MatPath{19} = 'model_pose.position';
info.MatPath{20} = 'model_pose.position.x';
info.MatPath{21} = 'model_pose.position.y';
info.MatPath{22} = 'model_pose.position.z';
info.MatPath{23} = 'model_pose.orientation';
info.MatPath{24} = 'model_pose.orientation.x';
info.MatPath{25} = 'model_pose.orientation.y';
info.MatPath{26} = 'model_pose.orientation.z';
info.MatPath{27} = 'model_pose.orientation.w';
info.MatPath{28} = 'set_model_pose';
info.MatPath{29} = 'disable_physics_updates';