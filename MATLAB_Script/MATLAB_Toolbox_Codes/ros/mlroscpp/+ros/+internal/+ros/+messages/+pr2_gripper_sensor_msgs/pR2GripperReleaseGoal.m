function [data, info] = pR2GripperReleaseGoal
%PR2GripperReleaseGoal gives an empty data for pr2_gripper_sensor_msgs/PR2GripperReleaseGoal

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'pr2_gripper_sensor_msgs/PR2GripperReleaseGoal';
[data.Command, info.Command] = ros.internal.ros.messages.pr2_gripper_sensor_msgs.pR2GripperReleaseCommand;
info.Command.MLdataType = 'struct';
info.MessageType = 'pr2_gripper_sensor_msgs/PR2GripperReleaseGoal';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,10);
info.MatPath{1} = 'command';
info.MatPath{2} = 'command.event';
info.MatPath{3} = 'command.event.trigger_conditions';
info.MatPath{4} = 'command.event.FINGER_SIDE_IMPACT_OR_ACC';
info.MatPath{5} = 'command.event.SLIP_AND_ACC';
info.MatPath{6} = 'command.event.FINGER_SIDE_IMPACT_OR_SLIP_OR_ACC';
info.MatPath{7} = 'command.event.SLIP';
info.MatPath{8} = 'command.event.ACC';
info.MatPath{9} = 'command.event.acceleration_trigger_magnitude';
info.MatPath{10} = 'command.event.slip_trigger_magnitude';
