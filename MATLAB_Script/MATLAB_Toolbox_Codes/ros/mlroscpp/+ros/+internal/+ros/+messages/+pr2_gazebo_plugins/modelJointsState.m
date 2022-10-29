function [data, info] = modelJointsState
%ModelJointsState gives an empty data for pr2_gazebo_plugins/ModelJointsState

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'pr2_gazebo_plugins/ModelJointsState';
[data.ModelPose, info.ModelPose] = ros.internal.ros.messages.geometry_msgs.pose;
info.ModelPose.MLdataType = 'struct';
info.ModelPose.MaxLen = NaN;
info.ModelPose.MinLen = 0;
data.ModelPose = data.ModelPose([],1);
[data.JointNames, info.JointNames] = ros.internal.ros.messages.ros.char('string',NaN);
[data.JointPositions, info.JointPositions] = ros.internal.ros.messages.ros.default_type('double',NaN);
info.MessageType = 'pr2_gazebo_plugins/ModelJointsState';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,12);
info.MatPath{1} = 'model_pose';
info.MatPath{2} = 'model_pose.position';
info.MatPath{3} = 'model_pose.position.x';
info.MatPath{4} = 'model_pose.position.y';
info.MatPath{5} = 'model_pose.position.z';
info.MatPath{6} = 'model_pose.orientation';
info.MatPath{7} = 'model_pose.orientation.x';
info.MatPath{8} = 'model_pose.orientation.y';
info.MatPath{9} = 'model_pose.orientation.z';
info.MatPath{10} = 'model_pose.orientation.w';
info.MatPath{11} = 'joint_names';
info.MatPath{12} = 'joint_positions';
