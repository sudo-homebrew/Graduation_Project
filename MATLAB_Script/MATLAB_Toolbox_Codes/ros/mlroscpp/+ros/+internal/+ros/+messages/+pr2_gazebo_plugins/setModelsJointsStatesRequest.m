function [data, info] = setModelsJointsStatesRequest
%SetModelsJointsStates gives an empty data for pr2_gazebo_plugins/SetModelsJointsStatesRequest

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'pr2_gazebo_plugins/SetModelsJointsStatesRequest';
[data.ModelNames, info.ModelNames] = ros.internal.ros.messages.ros.char('string',NaN);
[data.ModelJointsStates, info.ModelJointsStates] = ros.internal.ros.messages.pr2_gazebo_plugins.modelJointsState;
info.ModelJointsStates.MLdataType = 'struct';
info.ModelJointsStates.MaxLen = NaN;
info.ModelJointsStates.MinLen = 0;
data.ModelJointsStates = data.ModelJointsStates([],1);
info.MessageType = 'pr2_gazebo_plugins/SetModelsJointsStatesRequest';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,14);
info.MatPath{1} = 'model_names';
info.MatPath{2} = 'model_joints_states';
info.MatPath{3} = 'model_joints_states.model_pose';
info.MatPath{4} = 'model_joints_states.model_pose.position';
info.MatPath{5} = 'model_joints_states.model_pose.position.x';
info.MatPath{6} = 'model_joints_states.model_pose.position.y';
info.MatPath{7} = 'model_joints_states.model_pose.position.z';
info.MatPath{8} = 'model_joints_states.model_pose.orientation';
info.MatPath{9} = 'model_joints_states.model_pose.orientation.x';
info.MatPath{10} = 'model_joints_states.model_pose.orientation.y';
info.MatPath{11} = 'model_joints_states.model_pose.orientation.z';
info.MatPath{12} = 'model_joints_states.model_pose.orientation.w';
info.MatPath{13} = 'model_joints_states.joint_names';
info.MatPath{14} = 'model_joints_states.joint_positions';
