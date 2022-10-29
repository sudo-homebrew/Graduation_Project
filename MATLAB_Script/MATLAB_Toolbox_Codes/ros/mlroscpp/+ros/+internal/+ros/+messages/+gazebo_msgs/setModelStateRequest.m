function [data, info] = setModelStateRequest
%SetModelState gives an empty data for gazebo_msgs/SetModelStateRequest

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'gazebo_msgs/SetModelStateRequest';
[data.ModelState, info.ModelState] = ros.internal.ros.messages.gazebo_msgs.modelState;
info.ModelState.MLdataType = 'struct';
info.MessageType = 'gazebo_msgs/SetModelStateRequest';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,22);
info.MatPath{1} = 'model_state';
info.MatPath{2} = 'model_state.model_name';
info.MatPath{3} = 'model_state.pose';
info.MatPath{4} = 'model_state.pose.position';
info.MatPath{5} = 'model_state.pose.position.x';
info.MatPath{6} = 'model_state.pose.position.y';
info.MatPath{7} = 'model_state.pose.position.z';
info.MatPath{8} = 'model_state.pose.orientation';
info.MatPath{9} = 'model_state.pose.orientation.x';
info.MatPath{10} = 'model_state.pose.orientation.y';
info.MatPath{11} = 'model_state.pose.orientation.z';
info.MatPath{12} = 'model_state.pose.orientation.w';
info.MatPath{13} = 'model_state.twist';
info.MatPath{14} = 'model_state.twist.linear';
info.MatPath{15} = 'model_state.twist.linear.x';
info.MatPath{16} = 'model_state.twist.linear.y';
info.MatPath{17} = 'model_state.twist.linear.z';
info.MatPath{18} = 'model_state.twist.angular';
info.MatPath{19} = 'model_state.twist.angular.x';
info.MatPath{20} = 'model_state.twist.angular.y';
info.MatPath{21} = 'model_state.twist.angular.z';
info.MatPath{22} = 'model_state.reference_frame';
