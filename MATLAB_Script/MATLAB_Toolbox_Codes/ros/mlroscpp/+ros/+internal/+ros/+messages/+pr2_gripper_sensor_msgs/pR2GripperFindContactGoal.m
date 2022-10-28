function [data, info] = pR2GripperFindContactGoal
%PR2GripperFindContactGoal gives an empty data for pr2_gripper_sensor_msgs/PR2GripperFindContactGoal

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'pr2_gripper_sensor_msgs/PR2GripperFindContactGoal';
[data.Command, info.Command] = ros.internal.ros.messages.pr2_gripper_sensor_msgs.pR2GripperFindContactCommand;
info.Command.MLdataType = 'struct';
info.MessageType = 'pr2_gripper_sensor_msgs/PR2GripperFindContactGoal';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,7);
info.MatPath{1} = 'command';
info.MatPath{2} = 'command.zero_fingertip_sensors';
info.MatPath{3} = 'command.contact_conditions';
info.MatPath{4} = 'command.BOTH';
info.MatPath{5} = 'command.LEFT';
info.MatPath{6} = 'command.RIGHT';
info.MatPath{7} = 'command.EITHER';
