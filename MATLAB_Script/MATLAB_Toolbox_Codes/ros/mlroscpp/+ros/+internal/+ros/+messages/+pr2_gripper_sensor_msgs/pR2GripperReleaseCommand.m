function [data, info] = pR2GripperReleaseCommand
%PR2GripperReleaseCommand gives an empty data for pr2_gripper_sensor_msgs/PR2GripperReleaseCommand

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'pr2_gripper_sensor_msgs/PR2GripperReleaseCommand';
[data.Event, info.Event] = ros.internal.ros.messages.pr2_gripper_sensor_msgs.pR2GripperEventDetectorCommand;
info.Event.MLdataType = 'struct';
info.MessageType = 'pr2_gripper_sensor_msgs/PR2GripperReleaseCommand';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,9);
info.MatPath{1} = 'event';
info.MatPath{2} = 'event.trigger_conditions';
info.MatPath{3} = 'event.FINGER_SIDE_IMPACT_OR_ACC';
info.MatPath{4} = 'event.SLIP_AND_ACC';
info.MatPath{5} = 'event.FINGER_SIDE_IMPACT_OR_SLIP_OR_ACC';
info.MatPath{6} = 'event.SLIP';
info.MatPath{7} = 'event.ACC';
info.MatPath{8} = 'event.acceleration_trigger_magnitude';
info.MatPath{9} = 'event.slip_trigger_magnitude';
