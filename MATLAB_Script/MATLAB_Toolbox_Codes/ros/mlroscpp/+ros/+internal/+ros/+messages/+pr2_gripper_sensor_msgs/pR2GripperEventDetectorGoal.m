function [data, info] = pR2GripperEventDetectorGoal
%PR2GripperEventDetectorGoal gives an empty data for pr2_gripper_sensor_msgs/PR2GripperEventDetectorGoal

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'pr2_gripper_sensor_msgs/PR2GripperEventDetectorGoal';
[data.Command, info.Command] = ros.internal.ros.messages.pr2_gripper_sensor_msgs.pR2GripperEventDetectorCommand;
info.Command.MLdataType = 'struct';
info.MessageType = 'pr2_gripper_sensor_msgs/PR2GripperEventDetectorGoal';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,9);
info.MatPath{1} = 'command';
info.MatPath{2} = 'command.trigger_conditions';
info.MatPath{3} = 'command.FINGER_SIDE_IMPACT_OR_ACC';
info.MatPath{4} = 'command.SLIP_AND_ACC';
info.MatPath{5} = 'command.FINGER_SIDE_IMPACT_OR_SLIP_OR_ACC';
info.MatPath{6} = 'command.SLIP';
info.MatPath{7} = 'command.ACC';
info.MatPath{8} = 'command.acceleration_trigger_magnitude';
info.MatPath{9} = 'command.slip_trigger_magnitude';
