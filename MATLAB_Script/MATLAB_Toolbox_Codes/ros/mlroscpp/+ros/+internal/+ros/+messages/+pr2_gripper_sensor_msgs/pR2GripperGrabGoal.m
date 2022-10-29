function [data, info] = pR2GripperGrabGoal
%PR2GripperGrabGoal gives an empty data for pr2_gripper_sensor_msgs/PR2GripperGrabGoal

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'pr2_gripper_sensor_msgs/PR2GripperGrabGoal';
[data.Command, info.Command] = ros.internal.ros.messages.pr2_gripper_sensor_msgs.pR2GripperGrabCommand;
info.Command.MLdataType = 'struct';
info.MessageType = 'pr2_gripper_sensor_msgs/PR2GripperGrabGoal';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,2);
info.MatPath{1} = 'command';
info.MatPath{2} = 'command.hardness_gain';
