function [data, info] = setLinkStateRequest
%SetLinkState gives an empty data for gazebo_msgs/SetLinkStateRequest

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'gazebo_msgs/SetLinkStateRequest';
[data.LinkState, info.LinkState] = ros.internal.ros.messages.gazebo_msgs.linkState;
info.LinkState.MLdataType = 'struct';
info.MessageType = 'gazebo_msgs/SetLinkStateRequest';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,22);
info.MatPath{1} = 'link_state';
info.MatPath{2} = 'link_state.link_name';
info.MatPath{3} = 'link_state.pose';
info.MatPath{4} = 'link_state.pose.position';
info.MatPath{5} = 'link_state.pose.position.x';
info.MatPath{6} = 'link_state.pose.position.y';
info.MatPath{7} = 'link_state.pose.position.z';
info.MatPath{8} = 'link_state.pose.orientation';
info.MatPath{9} = 'link_state.pose.orientation.x';
info.MatPath{10} = 'link_state.pose.orientation.y';
info.MatPath{11} = 'link_state.pose.orientation.z';
info.MatPath{12} = 'link_state.pose.orientation.w';
info.MatPath{13} = 'link_state.twist';
info.MatPath{14} = 'link_state.twist.linear';
info.MatPath{15} = 'link_state.twist.linear.x';
info.MatPath{16} = 'link_state.twist.linear.y';
info.MatPath{17} = 'link_state.twist.linear.z';
info.MatPath{18} = 'link_state.twist.angular';
info.MatPath{19} = 'link_state.twist.angular.x';
info.MatPath{20} = 'link_state.twist.angular.y';
info.MatPath{21} = 'link_state.twist.angular.z';
info.MatPath{22} = 'link_state.reference_frame';
