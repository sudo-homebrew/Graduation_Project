function [data, info] = joints_data
%joints_data gives an empty data for sr_robot_msgs/joints_data

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'sr_robot_msgs/joints_data';
[data.JointsListLength, info.JointsListLength] = ros.internal.ros.messages.ros.default_type('int8',1);
[data.JointsList, info.JointsList] = ros.internal.ros.messages.sr_robot_msgs.joint;
info.JointsList.MLdataType = 'struct';
info.JointsList.MaxLen = NaN;
info.JointsList.MinLen = 0;
data.JointsList = data.JointsList([],1);
info.MessageType = 'sr_robot_msgs/joints_data';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,9);
info.MatPath{1} = 'joints_list_length';
info.MatPath{2} = 'joints_list';
info.MatPath{3} = 'joints_list.joint_name';
info.MatPath{4} = 'joints_list.joint_position';
info.MatPath{5} = 'joints_list.joint_target';
info.MatPath{6} = 'joints_list.joint_torque';
info.MatPath{7} = 'joints_list.joint_temperature';
info.MatPath{8} = 'joints_list.joint_current';
info.MatPath{9} = 'joints_list.error_flag';
