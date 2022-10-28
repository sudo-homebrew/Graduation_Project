function [data, info] = baseControllerState
%BaseControllerState gives an empty data for pr2_mechanism_controllers/BaseControllerState

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'pr2_mechanism_controllers/BaseControllerState';
[data.Command, info.Command] = ros.internal.ros.messages.geometry_msgs.twist;
info.Command.MLdataType = 'struct';
[data.JointVelocityMeasured, info.JointVelocityMeasured] = ros.internal.ros.messages.ros.default_type('double',NaN);
[data.JointVelocityCommanded, info.JointVelocityCommanded] = ros.internal.ros.messages.ros.default_type('double',NaN);
[data.JointVelocityError, info.JointVelocityError] = ros.internal.ros.messages.ros.default_type('double',NaN);
[data.JointEffortMeasured, info.JointEffortMeasured] = ros.internal.ros.messages.ros.default_type('double',NaN);
[data.JointEffortCommanded, info.JointEffortCommanded] = ros.internal.ros.messages.ros.default_type('double',NaN);
[data.JointEffortError, info.JointEffortError] = ros.internal.ros.messages.ros.default_type('double',NaN);
[data.JointNames, info.JointNames] = ros.internal.ros.messages.ros.char('string',NaN);
info.MessageType = 'pr2_mechanism_controllers/BaseControllerState';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,16);
info.MatPath{1} = 'command';
info.MatPath{2} = 'command.linear';
info.MatPath{3} = 'command.linear.x';
info.MatPath{4} = 'command.linear.y';
info.MatPath{5} = 'command.linear.z';
info.MatPath{6} = 'command.angular';
info.MatPath{7} = 'command.angular.x';
info.MatPath{8} = 'command.angular.y';
info.MatPath{9} = 'command.angular.z';
info.MatPath{10} = 'joint_velocity_measured';
info.MatPath{11} = 'joint_velocity_commanded';
info.MatPath{12} = 'joint_velocity_error';
info.MatPath{13} = 'joint_effort_measured';
info.MatPath{14} = 'joint_effort_commanded';
info.MatPath{15} = 'joint_effort_error';
info.MatPath{16} = 'joint_names';
