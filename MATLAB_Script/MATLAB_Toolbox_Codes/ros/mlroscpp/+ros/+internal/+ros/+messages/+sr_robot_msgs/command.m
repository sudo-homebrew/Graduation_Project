function [data, info] = command
%command gives an empty data for sr_robot_msgs/command

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'sr_robot_msgs/command';
[data.CommandType, info.CommandType] = ros.internal.ros.messages.ros.default_type('int8',1);
[data.SendupdateCommand, info.SendupdateCommand] = ros.internal.ros.messages.sr_robot_msgs.sendupdate;
info.SendupdateCommand.MLdataType = 'struct';
[data.ContrlrCommand, info.ContrlrCommand] = ros.internal.ros.messages.sr_robot_msgs.contrlr;
info.ContrlrCommand.MLdataType = 'struct';
info.MessageType = 'sr_robot_msgs/command';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,15);
info.MatPath{1} = 'command_type';
info.MatPath{2} = 'sendupdate_command';
info.MatPath{3} = 'sendupdate_command.sendupdate_length';
info.MatPath{4} = 'sendupdate_command.sendupdate_list';
info.MatPath{5} = 'sendupdate_command.sendupdate_list.joint_name';
info.MatPath{6} = 'sendupdate_command.sendupdate_list.joint_position';
info.MatPath{7} = 'sendupdate_command.sendupdate_list.joint_target';
info.MatPath{8} = 'sendupdate_command.sendupdate_list.joint_torque';
info.MatPath{9} = 'sendupdate_command.sendupdate_list.joint_temperature';
info.MatPath{10} = 'sendupdate_command.sendupdate_list.joint_current';
info.MatPath{11} = 'sendupdate_command.sendupdate_list.error_flag';
info.MatPath{12} = 'contrlr_command';
info.MatPath{13} = 'contrlr_command.contrlr_name';
info.MatPath{14} = 'contrlr_command.list_of_parameters';
info.MatPath{15} = 'contrlr_command.length_of_list';
